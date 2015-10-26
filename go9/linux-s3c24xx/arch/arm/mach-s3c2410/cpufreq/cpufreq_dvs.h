/*
 * cpufreq_dvs.h: DVS CPU clock scaling for s3c24xx family
 *
 * Copyright (C) 2008, TomTom International B.V.
 *
 * Author: Rogier Stam <rogier.stam@tomtom.com> 
 */

#define CPUFREQ_DVS_MEMORY_FREQ         132000
#define CPUFREQ_DVS_TRANSITION_LATENCY  1000
#define CPUFREQ_DVS_NR_FREQS            2
struct s3c24xx_dvs_handlers
{
        int     (*get)( void );
        void    (*set)( int state );
};
