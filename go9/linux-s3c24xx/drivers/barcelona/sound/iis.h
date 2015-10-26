#ifndef S3C2410_IIS_SND
#define S3C2410_IIS_SND

#define INTERNAL_MASTER_CLOCK	6000000

enum iis_cmd {
	IIS_IDLE_TX,
	IIS_IDLE_RX,
	IIS_ACTIVE_TX,
	IIS_ACTIVE_RX,
	IIS_ALIGN_TX,
	IIS_ALIGN_RX,
	IIS_FLUSH_RX,
	IIS_FLUSH_TX,
	IIS_START,
	IIS_STOP
};

int iis_init(unsigned int divider );
void iis_exit(void);

extern struct clk *iis_clock;
extern unsigned iis_clockrate;
extern unsigned iis_clk_div;	/* When in Internal Master mode 	*/

#ifdef CONFIG_CPU_S3C2412

typedef struct iis_hwintf {
	int  (*hw_init)(void);
	void (*hw_exit)(void);
	void (*control_cmd)(enum iis_cmd command);
	int  (*set_samplerate)(unsigned rate);
	int  (*set_samplesize)(unsigned size);
	void (*showbus)(const char *msg);
	void (*suspend)(void);
	void (*resume)(void);
} iis_hwintf_t;

extern iis_hwintf_t iis_hwintf;

static inline int  iis_hw_init(void) { return iis_hwintf.hw_init(); }
static inline void iis_hw_exit(void) { iis_hwintf.hw_exit(); }
static inline void iis_control_cmd(enum iis_cmd command) { return iis_hwintf.control_cmd(command); }
static inline int  iis_set_samplerate(unsigned rate) { return iis_hwintf.set_samplerate(rate); }
static inline int  iis_set_samplesize(unsigned size) { return iis_hwintf.set_samplesize(size); }
static inline void iis_showbus(const char *msg) { iis_hwintf.showbus(msg); }
static inline void iis_suspend(void) { iis_hwintf.suspend(); }
static inline void iis_resume(void) { iis_hwintf.resume(); }

#else /* CONFIG_CPU_S3C2412 */

int  iis_s3c2410_hw_init(void);
void iis_s3c2410_hw_exit(void);
void iis_s3c2410_control_cmd(enum iis_cmd command);
int  iis_s3c2410_set_samplerate(unsigned rate);
int  iis_s3c2410_set_samplesize(unsigned size);
void iis_s3c2410_showbus(const char *msg);
void iis_s3c2410_suspend(void);
void iis_s3c2410_resume(void);

static inline int  iis_hw_init(void) { return iis_s3c2410_hw_init(); }
static inline void iis_hw_exit(void) { iis_s3c2410_hw_exit(); }
static inline void iis_control_cmd(enum iis_cmd command) { return iis_s3c2410_control_cmd(command); }
static inline int  iis_set_samplerate(unsigned rate) { return iis_s3c2410_set_samplerate(rate); }
static inline int  iis_set_samplesize(unsigned size) { return iis_s3c2410_set_samplesize(size); }
static inline void iis_showbus(const char *msg) { iis_s3c2410_showbus(msg); }
static inline void iis_suspend(void) { iis_s3c2410_suspend(); }
static inline void iis_resume(void) { iis_s3c2410_resume(); }

#endif /* CONFIG_CPU_S3C2412 */

#endif /* S3C2410_IIS_SND */
