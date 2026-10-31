/* QNX Smart Traffic Controller
 * Real-time T-junction signal controller: Webster adaptive green timing, emergency vehicle
 * pre-emption, conflict-guarded signal outputs and a TCP command interface.
 * Tasks: controller, timer service, sampling, analytics, emergency, TCP server, logger. */

#define _GNU_SOURCE

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>  /* mlockall for RT scheduling */
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* Controller timing (seconds) and tuning constants */
#define DEFAULT_PORT      5000
#define LOST_TIME_S       8  /* Webster lost time: 2 phases x (yellow 3 + all-red 1) */
#define MIN_CYCLE_S       30
#define MAX_CYCLE_S       120
#define MIN_GREEN_S       7
#define MAX_GREEN_S       90
#define SAT_COUNT         40.0  /* vehicles per count window that mean saturated (tunable) */
#define MAX_Y             0.90  /* cap on total flow ratio keeps the cycle finite */
#define YELLOW_S          3
#define ALL_RED_S         1
#define EMERGENCY_ALL_RED_S 5
#define EMERGENCY_HOLD_S  10
#define FAILSAFE_S        3
#define FALLBACK_AFTER_S  15  /* no plan for this long -> fixed fallback timing */
#define FALLBACK_GREEN_S  10
#define MAX_COUNT         200
#define SAMPLE_PERIOD_MS  100  /* vehicle-sampling period */
#define CLIENT_RCV_TMO_S  2  /* a silent client cannot block the server longer than this */

/* SCHED_FIFO thread priorities (higher runs first) */
#define PRIO_CONTROLLER   30
#define PRIO_EMERGENCY    30
#define PRIO_TIMER        29
#define PRIO_ANALYTICS    20
#define PRIO_TCP          15
#define PRIO_LOGGER       10

/* POSIX message queues that connect the tasks */
#define MQ_CTRL_NAME      "/tc_ctrl"
#define MQ_ANALYTICS_NAME "/tc_analytics"
#define MQ_DEPTH          32

/* Signal controller states */
typedef enum { ST_GREEN, ST_YELLOW, ST_ALL_RED, ST_EMERGENCY_GREEN, ST_FAILSAFE } state_t;
static const char *STATE_NAME[] = { "GREEN", "YELLOW", "ALL_RED", "EMERGENCY_GREEN", "FAILSAFE" };

/* Events delivered to the controller through its message queue */
typedef enum {
    EV_STATE_TIMEOUT,
    EV_EMERG_REQUEST,
    EV_PLAN,
    EV_WATCHDOG,
    EV_SHUTDOWN
} ev_type_t;

typedef struct {
    ev_type_t type;
    int       a, b;  /* PLAN: g0,g1 | EMERG_REQUEST: a = 'N'/'S'/'E' | STATE_TIMEOUT: a = timer generation */
    int64_t   t_due_ns;
    int64_t   t_sent_ns;
} event_t;

typedef struct { int n, s, e; int64_t t_rx_ns; } counts_msg_t;

/* Monotonic clock helpers */
static inline int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static inline struct timespec ns_to_ts(int64_t ns)
{
    struct timespec ts;
    ts.tv_sec = ns / 1000000000LL;
    ts.tv_nsec = ns % 1000000000LL;
    return ts;
}

/* Latency statistics: 50 us histogram buckets plus min/avg/max and deadline-miss count */
#define HIST_BUCKETS 64
#define HIST_STEP_US 50

typedef struct {
    pthread_mutex_t lock;
    uint64_t count, over_deadline;
    int64_t  min_ns, max_ns, sum_ns;
    uint32_t hist[HIST_BUCKETS];
    int64_t  budget_ns;
} lat_stat_t;

static void lat_init(lat_stat_t *l, int64_t budget_ns)
{
    memset(l, 0, sizeof *l);
    pthread_mutex_init(&l->lock, NULL);
    l->min_ns = INT64_MAX;
    l->budget_ns = budget_ns;
}

static void lat_add(lat_stat_t *l, int64_t ns)
{
    if (ns < 0) ns = 0;
    pthread_mutex_lock(&l->lock);
    l->count++;
    l->sum_ns += ns;
    if (ns < l->min_ns) l->min_ns = ns;
    if (ns > l->max_ns) l->max_ns = ns;
    if (ns > l->budget_ns) l->over_deadline++;
    int b = (int)(ns / 1000 / HIST_STEP_US);
    if (b >= HIST_BUCKETS) b = HIST_BUCKETS - 1;
    l->hist[b]++;
    pthread_mutex_unlock(&l->lock);
}

/* Percentile in microseconds from the histogram (upper edge of the bucket) */
static double lat_pct_us(const lat_stat_t *l, double p)
{
    uint64_t target = (uint64_t)((double)l->count * p / 100.0 + 0.999999), acc = 0;
    if (l->count == 0) return 0.0;
    for (int i = 0; i < HIST_BUCKETS; i++) {
        acc += l->hist[i];
        if (acc >= target) {
            if (i == HIST_BUCKETS - 1) return (double)l->max_ns / 1000.0;
            return (double)((i + 1) * HIST_STEP_US);
        }
    }
    return (double)l->max_ns / 1000.0;
}

static void lat_format(lat_stat_t *l, const char *name, char *out, size_t n)
{
    pthread_mutex_lock(&l->lock);
    if (l->count == 0) {
        snprintf(out, n, "%s n=0", name);
    } else {
        snprintf(out, n, "%s n=%llu min_us=%.1f avg_us=%.1f p50_us=%.0f p99_us=%.0f max_us=%.1f miss=%llu(>%lldus)",
                 name, (unsigned long long)l->count, l->min_ns / 1000.0,
                 (double)l->sum_ns / (double)l->count / 1000.0,
                 lat_pct_us(l, 50), lat_pct_us(l, 99), l->max_ns / 1000.0,
                 (unsigned long long)l->over_deadline, (long long)(l->budget_ns / 1000));
    }
    pthread_mutex_unlock(&l->lock);
}

typedef struct {
    pthread_mutex_t lock;
    state_t state;
    char    n, s, e;
    int     g0, g1;
    int     fallback;
    char    em_active;
    int     phase;
    uint64_t conflicts_blocked;
    uint64_t transitions;
} status_t;

/* Status snapshot: written by the controller, read by the TCP thread (lights are 'R' 'Y' 'G') */
static status_t g_status = { .lock = PTHREAD_MUTEX_INITIALIZER, .state = ST_GREEN, .n = 'G', .s = 'G',
                             .e = 'R', .g0 = 30, .g1 = 30, .fallback = 0, .em_active = '-', .phase = 0 };

/* Queues, run flag and the latency trackers that feed STATS */
static mqd_t g_mq_ctrl = (mqd_t)-1, g_mq_an = (mqd_t)-1;
static mqd_t g_tx_ctrl = (mqd_t)-1, g_tx_an = (mqd_t)-1, g_tx_em = (mqd_t)-1;
static volatile sig_atomic_t g_running = 1;
static int  g_quiet = 0, g_use_rt = 1, g_port = DEFAULT_PORT;
static int  g_listen_fd = -1;
static lat_stat_t g_lat_event;  /* event queued -> controller starts handling it */
static lat_stat_t g_lat_timer;  /* timer due time -> controller handling (cycle-timer jitter) */
static lat_stat_t g_lat_emerg;  /* EMERG received on TCP -> yellow light (emergency response) */
static lat_stat_t g_lat_light;  /* controller handling -> lights written (decision-to-actuation) */
static lat_stat_t g_lat_e2g;  /* EMERG received -> emergency green shown (end to end) */
static lat_stat_t g_lat_sample;  /* sampling timer due -> sampling thread running (sampling jitter) */

/* Logger: ring buffer so real-time threads never block on stdout */
#define LOG_SLOTS 256
#define LOG_LEN   200

static char            g_logbuf[LOG_SLOTS][LOG_LEN];
static volatile unsigned g_log_head = 0, g_log_tail = 0;
static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_log_cond = PTHREAD_COND_INITIALIZER;
static uint64_t        g_log_dropped = 0;
static void logf_(const char *fmt, ...)
{
    char tmp[LOG_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (pthread_mutex_trylock(&g_log_lock) != 0) { g_log_dropped++; return; }  /* never wait in an RT thread */
    unsigned next = (g_log_head + 1) % LOG_SLOTS;
    if (next == g_log_tail) { g_log_dropped++; }
    else {
        memcpy(g_logbuf[g_log_head], tmp, sizeof tmp);
        g_log_head = next;
        pthread_cond_signal(&g_log_cond);
    }
    pthread_mutex_unlock(&g_log_lock);
}

static void *logger_thread(void *arg)
{
    (void)arg;
    int64_t last_stats = now_ns();
    while (g_running) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        pthread_mutex_lock(&g_log_lock);
        while (g_log_head == g_log_tail && g_running) {
            if (pthread_cond_timedwait(&g_log_cond, &g_log_lock, &ts) != 0) break;
        }
        while (g_log_tail != g_log_head) {
            char line[LOG_LEN];
            memcpy(line, g_logbuf[g_log_tail], LOG_LEN);
            g_log_tail = (g_log_tail + 1) % LOG_SLOTS;
            pthread_mutex_unlock(&g_log_lock);
            fputs(line, stdout);
            fputc('\n', stdout);
            pthread_mutex_lock(&g_log_lock);
        }
        pthread_mutex_unlock(&g_log_lock);
        fflush(stdout);
        if (now_ns() - last_stats > 30LL * 1000000000LL) {  /* print statistics every 30 s */
            last_stats = now_ns();
            char b[256];
            lat_format(&g_lat_event, "event_latency", b, sizeof b);  printf("[STATS] %s\n", b);
            lat_format(&g_lat_timer, "cycle_timer_jitter", b, sizeof b); printf("[STATS] %s\n", b);
            lat_format(&g_lat_emerg, "emergency_response", b, sizeof b); printf("[STATS] %s\n", b);
            fflush(stdout);
        }
    }
    return NULL;
}

/* Simulation-only output: signal states are reported through TCP STATUS; no GPIO pins are used */
static void lights_output(char n, char s, char e)
{
    (void)n;
    (void)s;
    (void)e;
}

/* Create a thread with SCHED_FIFO priority; falls back to default scheduling without privilege */
static int spawn(pthread_t *t, void *(*fn)(void *), int prio, const char *name)
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (g_use_rt) {
        struct sched_param sp = { .sched_priority = prio };
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &sp);
    }
    int rc = pthread_create(t, &attr, fn, NULL);
    if (rc != 0 && g_use_rt) {
        fprintf(stderr, "[RT] %s: SCHED_FIFO %d refused (%s); using default scheduling\n", name, prio, strerror(rc));
        pthread_attr_destroy(&attr);
        pthread_attr_init(&attr);
        rc = pthread_create(t, &attr, fn, NULL);
    }
    pthread_attr_destroy(&attr);
#if defined(__linux__) || defined(__QNXNTO__)
    if (rc == 0) pthread_setname_np(*t, name);
#endif
    return rc;
}

/* Queue an event for the controller */
static int ctrl_send(const event_t *ev, unsigned prio)
{
    return mq_send(g_tx_ctrl, (const char *)ev, sizeof *ev, prio);
}

/* Timer service: one thread sleeps on the earliest absolute CLOCK_MONOTONIC deadline.
 *   TM_CYCLE      signal state timeout (one shot, re-armed by the controller each state)
 *   TM_EMERGENCY  emergency-green timeout (one shot)
 *   TM_SAMPLE     periodic vehicle sampling, also runs the plan watchdog */
enum { TM_CYCLE, TM_EMERGENCY, TM_SAMPLE, TM_COUNT };
static struct {
    pthread_mutex_t lock;
    pthread_cond_t  cond;
    int64_t  due[TM_COUNT];  /* 0 = disarmed */
    int      gen[TM_COUNT];  /* bumped on every re-arm so stale timeouts can be ignored */
} g_tm = { .lock = PTHREAD_MUTEX_INITIALIZER };

static void timer_arm(int which, int64_t due_ns)
{
    pthread_mutex_lock(&g_tm.lock);
    g_tm.due[which] = due_ns;
    g_tm.gen[which]++;
    pthread_cond_signal(&g_tm.cond);
    pthread_mutex_unlock(&g_tm.lock);
}

static int timer_gen(int which)
{
    pthread_mutex_lock(&g_tm.lock);
    int g = g_tm.gen[which];
    pthread_mutex_unlock(&g_tm.lock);
    return g;
}

static int64_t timer_due(int which)
{
    pthread_mutex_lock(&g_tm.lock);
    int64_t due = g_tm.due[which];
    pthread_mutex_unlock(&g_tm.lock);
    return due;
}

static volatile int64_t g_last_plan_ns;
static volatile int     g_sample_seq;
static int64_t          g_sample_due_ns;
static pthread_mutex_t  g_sample_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   g_sample_cond = PTHREAD_COND_INITIALIZER;
static int64_t          g_sample_wake_ns;
static void *timer_thread(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_tm.lock);
    while (g_running) {
        int64_t earliest = 0;
        for (int i = 0; i < TM_COUNT; i++)
            if (g_tm.due[i] && (earliest == 0 || g_tm.due[i] < earliest)) earliest = g_tm.due[i];
        if (earliest == 0) { pthread_cond_wait(&g_tm.cond, &g_tm.lock); continue; }
        struct timespec ts = ns_to_ts(earliest);
        int rc = pthread_cond_timedwait(&g_tm.cond, &g_tm.lock, &ts);
        if (rc != ETIMEDOUT) continue;  /* re-armed or woken: recompute earliest deadline */
        int64_t t = now_ns();
        for (int i = 0; i < TM_COUNT; i++) {
            if (!g_tm.due[i] || g_tm.due[i] > t) continue;
            int64_t due = g_tm.due[i];
            event_t ev = { .t_due_ns = due, .t_sent_ns = t };
            if (i == TM_CYCLE || i == TM_EMERGENCY) {
                g_tm.due[i] = 0;
                ev.type = EV_STATE_TIMEOUT;
                ev.a = g_tm.gen[i];
                ev.b = i;
                pthread_mutex_unlock(&g_tm.lock);
                if (ctrl_send(&ev, 31) != 0) logf_("[TIMER] mq_send failed: %s", strerror(errno));
                pthread_mutex_lock(&g_tm.lock);
            } else {
                /* periodic sampling: re-arm and wake the sampling thread */
                g_tm.due[i] = due + (int64_t)SAMPLE_PERIOD_MS * 1000000LL;
                if (g_tm.due[i] <= t) g_tm.due[i] = t + (int64_t)SAMPLE_PERIOD_MS * 1000000LL;
                pthread_mutex_unlock(&g_tm.lock);
                pthread_mutex_lock(&g_sample_lock);
                g_sample_due_ns = due;
                g_sample_wake_ns = t;
                g_sample_seq++;
                pthread_cond_signal(&g_sample_cond);
                pthread_mutex_unlock(&g_sample_lock);
                /* watchdog: no plan for FALLBACK_AFTER_S -> controller switches to fallback timing */
                if (t - g_last_plan_ns >= (int64_t)FALLBACK_AFTER_S * 1000000000LL) {
                    event_t wd = { .type = EV_WATCHDOG, .t_due_ns = g_last_plan_ns + (int64_t)FALLBACK_AFTER_S * 1000000000LL,
                                   .t_sent_ns = t };
                    g_last_plan_ns = t;  /* re-trigger only once per window */
                    ctrl_send(&wd, 20);
                }
                pthread_mutex_lock(&g_tm.lock);
            }
        }
    }
    pthread_mutex_unlock(&g_tm.lock);
    return NULL;
}

/* Vehicle-sampling task: measures sampling-timer jitter */
static void *sampling_thread(void *arg)
{
    (void)arg;
    int seen = 0;
    pthread_mutex_lock(&g_sample_lock);
    while (g_running) {
        while (seen == g_sample_seq && g_running) pthread_cond_wait(&g_sample_cond, &g_sample_lock);
        seen = g_sample_seq;
        int64_t due = g_sample_due_ns;
        pthread_mutex_unlock(&g_sample_lock);
        lat_add(&g_lat_sample, now_ns() - due);
        pthread_mutex_lock(&g_sample_lock);
    }
    pthread_mutex_unlock(&g_sample_lock);
    return NULL;
}

/* Webster's method:  y0 = max(N,S)/SAT, y1 = E/SAT, Y = y0+y1
 *   C0 = (1.5 L + 5) / (1 - Y)   optimum cycle
 *   g_i = (C0 - L) * y_i / Y     green of phase i
 * Phase 0 = North+South together, phase 1 = East. */
static volatile int    g_web_cycle = 68;
static volatile double g_web_Y = 0.0;
static void calculate_split(int n, int s, int e, int *g0, int *g1)
{
    double y0 = (n > s ? n : s) / SAT_COUNT, y1 = e / SAT_COUNT, Y = y0 + y1;
    if (Y > MAX_Y) { y0 *= MAX_Y / Y; y1 *= MAX_Y / Y; Y = MAX_Y; }
    if (Y < 0.05) {  /* almost no traffic: short fixed cycle */
        *g0 = *g1 = (MIN_CYCLE_S - LOST_TIME_S) / 2;
        g_web_cycle = MIN_CYCLE_S; g_web_Y = Y;
        return;
    }
    double C = (1.5 * LOST_TIME_S + 5.0) / (1.0 - Y);
    if (C < MIN_CYCLE_S) C = MIN_CYCLE_S;
    if (C > MAX_CYCLE_S) C = MAX_CYCLE_S;
    double eff = C - LOST_TIME_S;
    int a = (int)(eff * y0 / Y + 0.5), b = (int)(eff * y1 / Y + 0.5);
    if (a < MIN_GREEN_S) a = MIN_GREEN_S;
    if (b < MIN_GREEN_S) b = MIN_GREEN_S;
    if (a > MAX_GREEN_S) a = MAX_GREEN_S;
    if (b > MAX_GREEN_S) b = MAX_GREEN_S;
    *g0 = a; *g1 = b;
    g_web_cycle = a + b + LOST_TIME_S; g_web_Y = Y;
}

/* Analytics task: turns vehicle counts into a green split and sends it as EV_PLAN */
static void *analytics_thread(void *arg)
{
    (void)arg;
    counts_msg_t m;
    while (g_running) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        ssize_t r = mq_timedreceive(g_mq_an, (char *)&m, sizeof m, NULL, &ts);
        if (r != (ssize_t)sizeof m) continue;
        int g0, g1;
        calculate_split(m.n, m.s, m.e, &g0, &g1);
        event_t ev = { .type = EV_PLAN, .a = g0, .b = g1, .t_sent_ns = now_ns() };
        g_last_plan_ns = ev.t_sent_ns;
        ctrl_send(&ev, 10);
        logf_("[WEBSTER] counts N=%d S=%d E=%d -> Y=%.2f cycle=%ds G0=%d G1=%d", m.n, m.s, m.e, g_web_Y, g_web_cycle, g0, g1);
    }
    return NULL;
}

/* Emergency task: forwards validated EMERG requests to the controller */
static mqd_t g_mq_em = (mqd_t)-1;
typedef struct { char approach; int64_t t_rx_ns; } emerg_msg_t;
static void *emergency_thread(void *arg)
{
    (void)arg;
    emerg_msg_t m;
    while (g_running) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        ssize_t r = mq_timedreceive(g_mq_em, (char *)&m, sizeof m, NULL, &ts);
        if (r != (ssize_t)sizeof m) continue;
        event_t ev = { .type = EV_EMERG_REQUEST, .a = m.approach, .t_due_ns = m.t_rx_ns, .t_sent_ns = now_ns() };
        ctrl_send(&ev, 30);  /* high mq priority: overtakes queued plan events */
        logf_("[EMERGENCY] request forwarded for approach %c", m.approach);
    }
    return NULL;
}

typedef struct {
    state_t state;
    int     phase;
    int     resume_phase;  /* phase and time left to return to after an emergency */
    int     resume_after;
    int     resume_seconds;
    int     em_extra_count;
    int     em_pending_extra;
    char    em_active, em_pending;
    int     g0, g1, fallback;
    int64_t pend_rx_ns, act_rx_ns;  /* arrival time of the pending / active emergency request */
} ctl_t;

/* Map the controller state to the R/Y/G light of each approach */
static void compute_lights(const ctl_t *c, char *n, char *s, char *e)
{
    *n = *s = *e = 'R';
    switch (c->state) {
    case ST_GREEN:
        if (c->phase == 0) *n = *s = 'G'; else *e = 'G';
        break;
    case ST_YELLOW:
        if (c->phase == 0) *n = *s = 'Y'; else *e = 'Y';
        break;
    case ST_EMERGENCY_GREEN:
        if (c->em_active == 'N') *n = 'G';
        else if (c->em_active == 'S') *s = 'G';
        else if (c->em_active == 'E') *e = 'G';
        break;
    default: break;
    }
}

/* Conflict: East shows a light while North or South does */
static int has_conflict(char n, char s, char e)
{
    return (e != 'R') && (n != 'R' || s != 'R');
}

/* Publish lights and status; the conflict guard sits between the state machine and the outputs */
static void publish(ctl_t *c, int64_t t_handle_ns)
{
    char n, s, e;
    compute_lights(c, &n, &s, &e);
    int conflict = has_conflict(n, s, e);
    /* unsafe combination: force all-red failsafe and arm the failsafe timer */
    if (conflict) {
        n = s = e = 'R';
        c->state = ST_FAILSAFE;
        c->em_active = '-';
        timer_arm(TM_EMERGENCY, 0);
        timer_arm(TM_CYCLE, now_ns() + (int64_t)FAILSAFE_S * 1000000000LL);
        logf_("[SAFETY] CONFLICT detected; forcing ALL-RED failsafe");
    }
    lights_output(n, s, e);
    lat_add(&g_lat_light, now_ns() - t_handle_ns);
    pthread_mutex_lock(&g_status.lock);
    g_status.state = c->state;
    g_status.n = n; g_status.s = s; g_status.e = e;
    g_status.g0 = c->g0; g_status.g1 = c->g1;
    g_status.fallback = c->fallback;
    g_status.em_active = c->em_active;
    g_status.phase = c->phase;
    g_status.transitions++;
    if (conflict) g_status.conflicts_blocked++;
    pthread_mutex_unlock(&g_status.lock);
    if (!g_quiet) logf_("[SIGNAL] state=%s phase=%d N=%c S=%c E=%c G0=%d G1=%d fallback=%d emergency=%c",
          STATE_NAME[c->state], c->phase, n, s, e, c->g0, c->g1, c->fallback, c->em_active);
}

/* Switch state, arm its timer and publish the new lights */
static void enter(ctl_t *c, state_t st, int duration_s, int64_t t_handle_ns)
{
    c->state = st;
    int64_t due = now_ns() + (int64_t)duration_s * 1000000000LL;
    /* emergency green uses the emergency timer, every other state the cycle timer */
    if (st == ST_EMERGENCY_GREEN) { timer_arm(TM_CYCLE, 0); timer_arm(TM_EMERGENCY, due); }
    else                          { timer_arm(TM_EMERGENCY, 0); timer_arm(TM_CYCLE, due); }
    publish(c, t_handle_ns);
}

/* Is a request waiting that must be served before the next normal phase? If so, make it active */
static int promote_pending(ctl_t *c)
{
    if (c->em_pending == '-') return 0;
    c->em_active = c->em_pending;
    c->em_pending = '-';
    c->em_extra_count = c->em_pending_extra;
    c->em_pending_extra = 0;
    c->act_rx_ns = c->pend_rx_ns;
    return 1;
}

/* Controller task: event-driven state machine GREEN -> YELLOW -> ALL_RED -> next phase or emergency green */
static void *controller_thread(void *arg)
{
    (void)arg;
    ctl_t c = { .state = ST_GREEN, .phase = 0, .em_active = '-', .em_pending = '-', .g0 = 30, .g1 = 30 };
    event_t ev;
    g_last_plan_ns = now_ns();
    enter(&c, ST_GREEN, c.g0, now_ns());
    while (g_running) {
        unsigned prio;
        ssize_t r = mq_receive(g_mq_ctrl, (char *)&ev, sizeof ev, &prio);  /* blocks until an event arrives */
        int64_t t = now_ns();
        if (r != (ssize_t)sizeof ev) { if (errno == EINTR) continue; if (!g_running) break; continue; }
        lat_add(&g_lat_event, t - ev.t_sent_ns);
        switch (ev.type) {
        case EV_SHUTDOWN:
            g_running = 0;
            break;
        /* apply the new green split from analytics */
        case EV_PLAN:
            c.g0 = ev.a; c.g1 = ev.b; c.fallback = 0;
            pthread_mutex_lock(&g_status.lock);
            g_status.g0 = c.g0; g_status.g1 = c.g1; g_status.fallback = 0;
            pthread_mutex_unlock(&g_status.lock);
            break;
        /* no plan received in time: switch to fixed fallback timing */
        case EV_WATCHDOG:
            if (!c.fallback) {
                c.fallback = 1;
                c.g0 = c.g1 = FALLBACK_GREEN_S;
                g_web_cycle = 2 * FALLBACK_GREEN_S + LOST_TIME_S;
                logf_("[WATCHDOG] no new traffic plan for %d seconds; fallback timing enabled", FALLBACK_AFTER_S);
                pthread_mutex_lock(&g_status.lock);
                g_status.g0 = c.g0; g_status.g1 = c.g1; g_status.fallback = 1;
                pthread_mutex_unlock(&g_status.lock);
            }
            break;
        /* emergency request: extend, queue, or start pre-emption */
case EV_EMERG_REQUEST: {
            char a = (char)ev.a;
            lat_add(&g_lat_emerg, t - ev.t_due_ns);
            /* same approach already being served: extend the window by 5 s */
            if (a == c.em_active && c.em_active != '-') {
                c.em_extra_count++;
                if (c.state == ST_EMERGENCY_GREEN) {
                    int64_t due = timer_due(TM_EMERGENCY);
                    if (due < t) due = t;
                    timer_arm(TM_EMERGENCY, due + 5000000000LL);
                }
                logf_("[EMERGENCY] additional ambulance on %c; +5 seconds", a);
                break;
            }
            /* otherwise queue it, grouping extra requests for the same pending approach */
            if (a == c.em_pending) {
                c.em_pending_extra++;
            } else if (c.em_pending == '-' || a == 'E') {
                c.em_pending = a;
                c.em_pending_extra = 0;
                c.pend_rx_ns = ev.t_due_ns;
            }
            /* signal is green: save the remaining time of this phase, then start yellow clearance */
            if (c.state == ST_GREEN) {
                c.resume_phase = c.phase;
                int64_t due = timer_due(TM_CYCLE);
                int rem = due > t ? (int)((due - t + 999999999LL) / 1000000000LL) : 0;
                c.resume_seconds = rem < 10 ? rem + 5 : rem;
                if (c.resume_seconds < 1) c.resume_seconds = 5;
                promote_pending(&c);
                enter(&c, ST_YELLOW, YELLOW_S, t);
            }
            break;
        }
        /* state timer expired: advance GREEN -> YELLOW -> ALL_RED -> next phase */
        case EV_STATE_TIMEOUT: {
            /* ignore stale timer events: a newer timer has been armed since */
            if (ev.a != timer_gen(ev.b)) break;
            lat_add(&g_lat_timer, t - ev.t_due_ns);
            switch (c.state) {
            case ST_GREEN:
                enter(&c, ST_YELLOW, YELLOW_S, t);
                break;
            case ST_YELLOW:
                /* normal phase clearance is 1 s all-red; emergency pre-emption needs 5 s */
                enter(&c, ST_ALL_RED,
                      (c.em_active != '-' || c.em_pending != '-') ? EMERGENCY_ALL_RED_S : ALL_RED_S, t);
                break;
            case ST_ALL_RED:
                /* request arrived during yellow/all-red: make it active */
                if (c.em_active == '-' && c.em_pending != '-') {
                    c.resume_phase = c.phase;
                    promote_pending(&c);
                }
                /* emergency active: give the ambulance its exclusive green */
                if (c.em_active != '-') {
                    enter(&c, ST_EMERGENCY_GREEN, EMERGENCY_HOLD_S + 5 * c.em_extra_count, t);
                    lat_add(&g_lat_e2g, now_ns() - c.act_rx_ns);  /* request -> green for the ambulance */
                } else if (c.resume_after) {  /* emergency just ended: resume the interrupted phase */
                    c.resume_after = 0;
                    c.phase = c.resume_phase;
                    enter(&c, ST_GREEN, c.resume_seconds > 0 ? c.resume_seconds : (c.phase == 0 ? c.g0 : c.g1), t);
                    c.resume_seconds = 0;
                } else {
                    c.phase = 1 - c.phase;
                    enter(&c, ST_GREEN, c.phase == 0 ? c.g0 : c.g1, t);
                }
                break;
            case ST_EMERGENCY_GREEN:
                c.em_active = '-';
                c.em_extra_count = 0;
                c.phase = c.resume_phase;
                /* emergency window over: return to the interrupted phase through all-red */
                c.resume_after = 1;
                enter(&c, ST_ALL_RED, EMERGENCY_ALL_RED_S, t);
                break;
            case ST_FAILSAFE:
                c.phase = 0;
                c.em_active = '-';
                enter(&c, ST_ALL_RED, ALL_RED_S, t);
                break;
            }
            break;
        }
        }
    }
    return NULL;
}

/* TCP interface: one newline-terminated command per connection */
static int send_line(int fd, const char *s)
{
    size_t len = strlen(s), off = 0;
    while (off < len) {
        ssize_t w = send(fd, s + off, len - off, MSG_NOSIGNAL);
        if (w <= 0) return -1;
        off += (size_t)w;
    }
    return 0;
}

/* Commands: PING, STATUS, COUNTS n s e, EMERG N|S|E, STATS */
static void handle_command(int fd, char *line)
{
    char out[512];
    if (strcmp(line, "PING") == 0) { send_line(fd, "PONG\n"); return; }
    if (strcmp(line, "STATUS") == 0) {
        pthread_mutex_lock(&g_status.lock);
        state_t st = g_status.state;
        char ln = g_status.n, ls = g_status.s, le = g_status.e, em = g_status.em_active;
        int g0 = g_status.g0, g1 = g_status.g1, fb = g_status.fallback;
        int cycle = (int)g_web_cycle;
        pthread_mutex_unlock(&g_status.lock);
        int which = (st == ST_EMERGENCY_GREEN) ? TM_EMERGENCY : TM_CYCLE;
        int64_t due = timer_due(which), now = now_ns();
        int rem = due > now ? (int)((due - now + 999999999LL) / 1000000000LL) : 0;
        snprintf(out, sizeof out, "STATE %s N=%c S=%c E=%c G0=%d G1=%d FB=%d EM=%c C=%d REM=%d\n",
                 STATE_NAME[st], ln, ls, le, g0, g1, fb, em, cycle, rem);
        send_line(fd, out);
        return;
    }
    if (strncmp(line, "COUNTS ", 7) == 0) {
        int n, s, e, used = 0;
        if (sscanf(line + 7, "%d %d %d %n", &n, &s, &e, &used) < 3 || line[7 + used] != '\0' ||
            n < 0 || s < 0 || e < 0 || n > MAX_COUNT || s > MAX_COUNT || e > MAX_COUNT) {
            send_line(fd, "ERR bad_counts\n");
            return;
        }
        counts_msg_t m = { n, s, e, now_ns() };
        if (mq_send(g_tx_an, (const char *)&m, sizeof m, 0) != 0) { send_line(fd, "ERR busy\n"); return; }
        int g0, g1;
        calculate_split(n, s, e, &g0, &g1);  /* same pure function the analytics task applies */
        snprintf(out, sizeof out, "PLAN %d %d %d %.2f\n", g0, g1, (int)g_web_cycle, (double)g_web_Y);
        send_line(fd, out);
        return;
    }
    if (strncmp(line, "EMERG ", 6) == 0) {
        char a = (char)((line[6] >= 'a' && line[6] <= 'z') ? line[6] - 32 : line[6]);
        if ((a != 'N' && a != 'S' && a != 'E') || line[7] != '\0' || strlen(line) != 7) {
            send_line(fd, "ERR bad_approach\n");
            return;
        }
        emerg_msg_t m = { a, now_ns() };
        if (mq_send(g_tx_em, (const char *)&m, sizeof m, 0) != 0) { send_line(fd, "ERR busy\n"); return; }
        send_line(fd, "OK\n");
        return;
    }
    if (strcmp(line, "STATS") == 0) {
        char b1[256], b2[256], b3[256], b4[256], b5[256], b6[256];
        lat_format(&g_lat_event, "event", b1, sizeof b1);
        lat_format(&g_lat_timer, "cycle_timer", b2, sizeof b2);
        lat_format(&g_lat_emerg, "emergency", b3, sizeof b3);
        lat_format(&g_lat_light, "actuation", b4, sizeof b4);
        lat_format(&g_lat_sample, "sampling", b5, sizeof b5);
        lat_format(&g_lat_e2g, "emerg_to_green", b6, sizeof b6);
        pthread_mutex_lock(&g_status.lock);
        uint64_t cb = g_status.conflicts_blocked, tr = g_status.transitions;
        pthread_mutex_unlock(&g_status.lock);
        char big[2000];
        snprintf(big, sizeof big, "STATS transitions=%llu conflicts_blocked=%llu | %s | %s | %s | %s | %s | %s\n",
                 (unsigned long long)tr, (unsigned long long)cb, b1, b2, b3, b4, b5, b6);
        send_line(fd, big);
        return;
    }
    send_line(fd, "ERR unknown_command\n");
}

/* Accept one client at a time: read one line, answer, close */
static void *tcp_thread(void *arg)
{
    (void)arg;
    while (g_running) {
        int fd = accept(g_listen_fd, NULL, NULL);
        if (fd < 0) { if (errno == EINTR) continue; if (!g_running) break; continue; }
        struct timeval tv = { CLIENT_RCV_TMO_S, 0 };  /* a silent client cannot block the server */
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        char buf[256];
        int len = 0;
        while (len < (int)sizeof buf - 1) {
            int r = (int)recv(fd, buf + len, sizeof buf - 1 - len, 0);
            if (r <= 0) break;
            len += r;
            if (memchr(buf, '\n', len)) break;
        }
        buf[len] = '\0';
        char *nl = strchr(buf, '\n');
        if (nl) {
            *nl = '\0';
            int l = (int)strlen(buf);
            while (l > 0 && (buf[l - 1] == '\r' || buf[l - 1] == ' ')) buf[--l] = '\0';
            handle_command(fd, buf);
        }
        close(fd);
    }
    return NULL;
}

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
    if (g_listen_fd >= 0) shutdown(g_listen_fd, SHUT_RDWR);
}

/* Create a queue with a blocking receive end and a non-blocking send end so senders never stall RT or TCP threads */
static mqd_t open_queue(const char *name, size_t msgsize, mqd_t *tx)
{
    struct mq_attr at = { .mq_flags = 0, .mq_maxmsg = MQ_DEPTH, .mq_msgsize = (long)msgsize, .mq_curmsgs = 0 };
    mq_unlink(name);
    mqd_t q = mq_open(name, O_CREAT | O_RDWR, 0600, &at);
    if (q == (mqd_t)-1 && errno == EINVAL) {  /* Linux non-root limit msg_max = 10: retry smaller */
        at.mq_maxmsg = 10;
        q = mq_open(name, O_CREAT | O_RDWR, 0600, &at);
    }
    if (q == (mqd_t)-1) {
        fprintf(stderr, "mq_open(%s) failed: %s%s\n", name, strerror(errno),
#ifdef __QNXNTO__
                "  (is the 'mqueue' manager running?  run:  mqueue &)");
#else
                "");
#endif
        exit(1);
    }
    *tx = mq_open(name, O_WRONLY | O_NONBLOCK);
    if (*tx == (mqd_t)-1) { fprintf(stderr, "mq_open(%s, tx) failed: %s\n", name, strerror(errno)); exit(1); }
    return q;
}

static void usage(const char *p)
{
    fprintf(stderr, "usage: %s [-p port] [-q] [-n]\n", p);
    exit(2);
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-p") && i + 1 < argc) g_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-q")) g_quiet = 1;
        else if (!strcmp(argv[i], "-n")) g_use_rt = 0;
        else usage(argv[0]);
    }
    if (g_port <= 0 || g_port > 65535) usage(argv[0]);
    struct sigaction sa = { .sa_handler = on_signal };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
    /* Deadlines: event 1 ms, timer 1 ms, emergency 20 ms, actuation 1 ms, sampling 1 ms, emergency-to-green 5 s */
    lat_init(&g_lat_event, 1000000);
    lat_init(&g_lat_timer, 1000000);
    lat_init(&g_lat_emerg, 20000000);
    lat_init(&g_lat_light, 1000000);
    lat_init(&g_lat_sample, 1000000);
    lat_init(&g_lat_e2g, 5000000000LL);
    if (g_use_rt) mlockall(MCL_CURRENT | MCL_FUTURE);  /* avoid page faults in the critical path */
    g_mq_ctrl = open_queue(MQ_CTRL_NAME, sizeof(event_t), &g_tx_ctrl);
    g_mq_an   = open_queue(MQ_ANALYTICS_NAME, sizeof(counts_msg_t), &g_tx_an);
    g_mq_em   = open_queue("/tc_emerg", sizeof(emerg_msg_t), &g_tx_em);
    pthread_condattr_t ca;  /* monotonic condvars: immune to wall-clock changes */
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&g_tm.cond, &ca);
    pthread_cond_init(&g_sample_cond, NULL);
    g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)g_port), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(g_listen_fd, (struct sockaddr *)&addr, sizeof addr) != 0 || listen(g_listen_fd, 8) != 0) {
        fprintf(stderr, "bind/listen on port %d failed: %s\n", g_port, strerror(errno));
        return 1;
    }
    /* Start every task with its SCHED_FIFO priority */
    pthread_t th_log, th_ctl, th_em, th_tm, th_an, th_tcp, th_smp;
    spawn(&th_log, logger_thread, PRIO_LOGGER, "logger");
    spawn(&th_ctl, controller_thread, PRIO_CONTROLLER, "signal_ctl");
    spawn(&th_em, emergency_thread, PRIO_EMERGENCY, "emergency");
    spawn(&th_tm, timer_thread, PRIO_TIMER, "timer_svc");
    spawn(&th_smp, sampling_thread, PRIO_ANALYTICS, "sampling");
    spawn(&th_an, analytics_thread, PRIO_ANALYTICS, "analytics");
    spawn(&th_tcp, tcp_thread, PRIO_TCP, "tcp_server");
    logf_("[TCP] controller listening on port %d  (realtime=%d)", g_port, g_use_rt);
    timer_arm(TM_SAMPLE, now_ns() + (int64_t)SAMPLE_PERIOD_MS * 1000000LL);
    while (g_running) pause();  /* main waits for SIGINT/SIGTERM */
    /* Shutdown: wake every blocked thread, join them, print final statistics, remove the queues */
    event_t down = { .type = EV_SHUTDOWN, .t_sent_ns = now_ns() };
    ctrl_send(&down, 31);
    pthread_mutex_lock(&g_tm.lock); pthread_cond_broadcast(&g_tm.cond); pthread_mutex_unlock(&g_tm.lock);
    pthread_mutex_lock(&g_sample_lock); pthread_cond_broadcast(&g_sample_cond); pthread_mutex_unlock(&g_sample_lock);
    pthread_mutex_lock(&g_log_lock); pthread_cond_broadcast(&g_log_cond); pthread_mutex_unlock(&g_log_lock);
    pthread_join(th_ctl, NULL); pthread_join(th_em, NULL); pthread_join(th_tm, NULL);
    pthread_join(th_smp, NULL); pthread_join(th_an, NULL); pthread_join(th_tcp, NULL); pthread_join(th_log, NULL);
    char b[256];
    lat_format(&g_lat_event, "event_latency", b, sizeof b);      printf("[FINAL] %s\n", b);
    lat_format(&g_lat_timer, "cycle_timer_jitter", b, sizeof b); printf("[FINAL] %s\n", b);
    lat_format(&g_lat_emerg, "emergency_response", b, sizeof b); printf("[FINAL] %s\n", b);
    lat_format(&g_lat_sample, "sampling_jitter", b, sizeof b);   printf("[FINAL] %s\n", b);
    lat_format(&g_lat_e2g, "emerg_to_green", b, sizeof b);       printf("[FINAL] %s\n", b);
    mq_close(g_mq_ctrl); mq_close(g_mq_an); mq_close(g_mq_em);
    mq_close(g_tx_ctrl); mq_close(g_tx_an); mq_close(g_tx_em);
    mq_unlink(MQ_CTRL_NAME); mq_unlink(MQ_ANALYTICS_NAME); mq_unlink("/tc_emerg");
    close(g_listen_fd);
    return 0;
}
