#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <termios.h>
#include <linux/socket.h>
#include <linux/un.h>
#include <hardware/gps.h>
#include <hardware/hardware.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "wl_log.h"

#define DRIVER_VERSION "WELINK_GPS_V1.0.0B01"
#define NMEA_PORT_PATH_CONFIG "/etc/NMEAPORT"
#define LEN_GPS_BUF (1024)
#define LEN_GPS_INFO (128)
#define  MAX_NMEA_INFO_SEG  (20)

#define ATCMD_ZGINIT (1)
#define ATCMD_ZGMODE_3 (2)
#define ATCMD_ZGNMEA_31 (3)
#define ATCMD_ZGNMEA_0 (4)
#define ATCMD_ZGRUN_0 (5)
#define ATCMD_ZGRUN_2 (6)
#define ATCMD_ZGFIXRATE_65535 (7)
#define ATCMD_ZGFIXRATE_1 (8)

//ÉùÃ÷
static int wl_gps_init(GpsCallbacks* callbacks);
static void wl_gps_cleanup(void);
static int wl_gps_start(void);
static int wl_gps_stop(void);
static int wl_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty);
static int wl_gps_inject_location(double latitude, double longitude, float accuracy);
static void wl_gps_delete_aiding_data(GpsAidingData flags);
static int wl_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
            uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time);
static const void* wl_gps_get_extension(const char* name);
static int wl_open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device);
static int wl_gps_xtra_init(GpsXtraCallbacks* callbacks);
static int wl_gps_xtra_inject_xtra_data(char* data, int length);
static void wl_read_port_thread(void *param);


typedef struct
{
    int len;
    char m_buf[LEN_GPS_BUF+1];
} gps_info_buf;

typedef struct 
{
    const char * m_beg;
    const char * m_end;
} Charseg;

typedef struct 
{
    int m_count;
    Charseg m_segs[ MAX_NMEA_INFO_SEG ];
} NmeaInfoSegs;

typedef struct
{
    int m_year;
    int m_month;
    int m_day;
    int m_sub;
} UtcInfo;

typedef struct
{
    unsigned int m_count;
    int m_number[32];
} UsingSatellitesInfo;

static gps_info_buf *g_gps_info_buf = NULL;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_nmea_fd = -1;
static GpsCallbacks *p_java_layer_callback = NULL;
static GpsXtraCallbacks* p_java_layer_xtra_callbacks = NULL;
static unsigned char g_is_internal_initialized = 0;
static GpsStatusValue g_cur_gps_status = GPS_STATUS_NONE;
static pthread_t g_read_port_thread;
static pthread_t g_read_buff_thread;
static pthread_t g_test;
static unsigned char g_need_reading_nmea = 0;
static UtcInfo g_utc_info;
static UsingSatellitesInfo g_satellites_info;
static unsigned char g_sv_status_flag = 0;
static GpsSvStatus g_sv_status_info;
static unsigned char g_cur_atcmd = 0;

static const GpsInterface  wl_GpsInterface = 
{
    sizeof(GpsInterface),
    //when system enable gps , it will be called
    /* 
     * Opens the interface and provides the callback routines 
     * to the implemenation of this interface. 
     */
    wl_gps_init,
    //when app use gps , this will be called second
    /*
     * Starts navigating.
     */  
    wl_gps_start,
    //when app exit , and does not use  gps , this will be called first
    /*
     * Stop navigating.
     */ 
    wl_gps_stop,
    //when system disable gps , it will be called
    /*
     * Close the interface.
     */ 
    wl_gps_cleanup,
    //
    /*
     * Injects the current time.
     */  
    wl_gps_inject_time,
    //
    /*
     * Injects current location from another location provider
     * (typically cell ID). 
     * latitude and longitude are measured in degrees 
     * expected accuracy is measured in meters 
     */  
    wl_gps_inject_location,
    //for Performance test
    /* 
     * Specifies that the next call to start will not use the
     * information defined in the flags. GPS_DELETE_ALL is passed for 
     * a cold start.
     */
    wl_gps_delete_aiding_data,
    //when app use gps , this will be called first
    /* 
     * min_interval represents the time between fixes in milliseconds. 
     * preferred_accuracy represents the requested fix accuracy in meters. 
     * preferred_time represents the requested time to first fix in milliseconds. 
     */ 
    wl_gps_set_position_mode,
    //
    /*
     * Get a pointer to extension information.
     */  
    wl_gps_get_extension,
};

static const GpsXtraInterface wl_GpsXtraInterface = 
{
    sizeof(GpsXtraInterface),
    wl_gps_xtra_init,
    wl_gps_xtra_inject_xtra_data
};


static struct hw_module_methods_t wl_gps_module_methods = 
{
    .open = wl_open_gps
};

struct hw_module_t HAL_MODULE_INFO_SYM = 
{
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "Welink Module",
    .author = "Thomas Yang",
    .methods = &wl_gps_module_methods,
};

static int wl_calc_utc_sub()
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));
    return time_local - time_utc;
}

static int wl_get_all_segments_from_buf( NmeaInfoSegs *seg, const char *line_buf, int line_len)
{
    int    count = 0;
    char *p = line_buf;
    char *line_end = line_buf+line_len;

    if ((p[0] != '$') || (p[line_len-3] != '*'))
    {
    	LOGD("p[%d]:%c",(line_len-3),p[line_len-3]);
        LOGD("Line format not correct");
        return 0;
    }

    p++;
    line_end -= 3;

    while (p < line_end) 
    {
        const char*  q = p;

        q = memchr(p, ',', line_end-p);
        if (q == NULL)
            q = line_end;
        
        if (q >= p) 
        {
            if (count < MAX_NMEA_INFO_SEG) 
            {
                seg->m_segs[count].m_beg = p;
                seg->m_segs[count].m_end = q;
                count += 1;
            }
        }
        
        if (q < line_end)
            q += 1;

        p = q;
    }

    seg->m_count = count;
    return count;
}

static Charseg wl_get_segments_by_index(NmeaInfoSegs *nmea_seg, int index)
{
    Charseg  seg;
    static const char *empty_str = "";

    if (index < 0 || index >= nmea_seg->m_count) 
    {
        seg.m_beg = seg.m_end = empty_str;
    } 
    else
    {
        seg = nmea_seg->m_segs[index];
    }

    return seg;
}

static int str2int(const char *beg, const char * end)
{
    int   result = 0;
    int   len    = end - beg;

    for ( ; len > 0; len--, beg++ )
    {
        int  c;

        if (beg >= end)
            goto Fail;

        c = *beg - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double str2float(const char *beg, const char *end)
{
    int   len    = end - beg;
    char  temp[16];
	double tmp;

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, beg, len );
    temp[len] = 0;
	tmp = strtod( temp, NULL );
    return strtod( temp, NULL );
}

static double wl_get_latlong_from_seg(Charseg seg)
{
    double  val     = str2float(seg.m_beg, seg.m_end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}

static int wl_get_latlong(GpsLocation *loc, Charseg latitude, Charseg latitudeHemi, Charseg longitude, Charseg longitudeHemi )
{
    double   lat, lon;
    Charseg seg;

    seg = latitude;
    if (seg.m_beg + 6 > seg.m_end) 
    {
        LOGD("Warning! Latitude is not correct");
        return -1;
    }
    
    lat = wl_get_latlong_from_seg(seg);
	LOGD("wl_get_latlong:lat=%f",lat);
    if (latitudeHemi.m_beg[0] == 'S')
        lat = -lat;

    seg = longitude;
    if (seg.m_beg + 6 > seg.m_end) 
    {
        LOGD("Warning! longitude is not correct");
        return -1;
    }
    lon = wl_get_latlong_from_seg(seg);
	LOGD("wl_get_latlong:lon=%f",lon);
    if (longitudeHemi.m_beg[0] == 'W')
        lon = -lon;

    loc->flags    |= GPS_LOCATION_HAS_LAT_LONG;
    loc->latitude  = lat;
    loc->longitude = lon;
    return 0;
}

static int wl_get_altitude(GpsLocation *loc, Charseg altitude, Charseg units)
{
    Charseg seg = altitude;

    if (seg.m_beg >= seg.m_end)
        return -1;

    loc->flags   |= GPS_LOCATION_HAS_ALTITUDE;
    loc->altitude = str2float(seg.m_beg, seg.m_end);
    return 0;
}

static int wl_get_time(GpsLocation *loc, Charseg  seg)
{
    int hour, minute;
    double seconds;
    struct tm  tm;
	struct tm  utc;
    time_t     fix_time;
	double tmp = 0 ;

    if (seg.m_beg + 6 > seg.m_end)
        return -1;

    if (g_utc_info.m_year < 0)
    {
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        g_utc_info.m_year = tm.tm_year + 1900;
        g_utc_info.m_month = tm.tm_mon + 1;
        g_utc_info.m_day  = tm.tm_mday;
    }

    hour = str2int(seg.m_beg,   seg.m_beg+2);
    minute  = str2int(seg.m_beg+2, seg.m_beg+4);
    seconds = str2float(seg.m_beg+4, seg.m_end);

    tm.tm_hour  = hour;
    tm.tm_min   = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year  = g_utc_info.m_year - 1900;
    tm.tm_mon   = g_utc_info.m_month - 1;
    tm.tm_mday  = g_utc_info.m_day;
    tm.tm_isdst = -1;

	tmp = ceil((fabs(loc->longitude) - 7.5) / 15 + 1);
	g_utc_info.m_sub = (loc->longitude / fabs(loc->longitude)) * tmp * 3600;

	fix_time = mktime( &tm ) + g_utc_info.m_sub;
	
    loc->timestamp = (long long)fix_time;
	
	gmtime_r( (time_t*) &loc->timestamp, &utc );
	
    return 0;
}

static int wl_get_bearing(GpsLocation *loc, Charseg bearing)
{
    Charseg seg = bearing;
    float   value = 0.0f;

    if (seg.m_beg < seg.m_end) 
    {
        value = str2float(seg.m_beg, seg.m_end);
    } 
    else if ((seg.m_beg >= seg.m_end) && (loc->flags & GPS_LOCATION_HAS_BEARING)) 
    {
        return 0;
    }

    loc->flags   |= GPS_LOCATION_HAS_BEARING;
    loc->bearing  = value;
    return 0;
}

static int wl_get_speed(GpsLocation *loc, Charseg speed)
{
    Charseg seg = speed;

    if (seg.m_beg >= seg.m_end)
        return -1;

    loc->flags  |= GPS_LOCATION_HAS_SPEED;
    loc->speed = (1.852 / 3.6) * str2float(seg.m_beg, seg.m_end); 
    return 0;
}

static int wl_get_date(GpsLocation *loc, Charseg date, Charseg time)
{
    Charseg  seg = date;
    int day, month, year;

    if (seg.m_beg + 6 != seg.m_end) 
    {
        LOGD("Date format error");
        return -1;
    }
    
    day  = str2int(seg.m_beg, seg.m_beg+2);
    month  = str2int(seg.m_beg+2, seg.m_beg+4);
    year = str2int(seg.m_beg+4, seg.m_beg+6) + 2000;

    if ((day|month|year) < 0) 
    {
        LOGD("Date format error");
        return -1;
    }

    g_utc_info.m_year  = year;
    g_utc_info.m_month   = month;
    g_utc_info.m_day   = day;

    return wl_get_time(loc, time);
}

static void wl_parse_nmea_line(GpsLocation *loc, char *line_buf, int line_len)
{
    NmeaInfoSegs info_segs[1];
    Charseg seg;
        
    if (line_len < 9)
    {
        LOGD("Len=%d, too short for a nmea line", line_len);
        return;
    }

    if (0 == wl_get_all_segments_from_buf(info_segs, line_buf, line_len))
    {
        LOGD("No valid segments get");
        return;
    }

    seg = wl_get_segments_by_index(info_segs, 0);
    if (seg.m_beg + 5 > seg.m_end)
    {
        LOGD("NMEA Token string too short");
        return;
    }

    seg.m_beg += 2;

    if ((!memcmp(seg.m_beg, "GGA", 3))
        || (!memcmp(seg.m_beg, "GSA", 3))
        || (!memcmp(seg.m_beg, "GSV", 3))
        || (!memcmp(seg.m_beg, "VTG", 3))
        || (!memcmp(seg.m_beg, "RMC", 3) ) )
    {
        if (p_java_layer_callback->nmea_cb)
        {
            time_t cur_time = time(NULL);
            p_java_layer_callback->nmea_cb(cur_time, line_buf, line_len);
        }
    }
    else
    {
        LOGD("Not a correct NMEA line: %.*s", line_len, line_buf);
        return;
    }

    if (!memcmp(seg.m_beg, "GGA", 3))
    {
        Charseg  seg_time          = wl_get_segments_by_index(info_segs,1);
        Charseg  seg_latitude      = wl_get_segments_by_index(info_segs,2);
        Charseg  seg_latitudeHemi  = wl_get_segments_by_index(info_segs,3);
        Charseg  seg_longitude     = wl_get_segments_by_index(info_segs,4);
        Charseg  seg_longitudeHemi = wl_get_segments_by_index(info_segs,5);
        Charseg  seg_altitude      = wl_get_segments_by_index(info_segs,9);
        Charseg  seg_altitudeUnits = wl_get_segments_by_index(info_segs,10);

        wl_get_latlong(loc, seg_latitude, seg_latitudeHemi, seg_longitude, seg_longitudeHemi);
        wl_get_altitude(loc, seg_altitude, seg_altitudeUnits);
        wl_get_time(loc, seg_time);
    }
    else if (!memcmp(seg.m_beg, "GSA", 3))
    {
        Charseg seg_acc = wl_get_segments_by_index(info_segs, 15);

        loc->accuracy = str2float(seg_acc.m_beg, seg_acc.m_end);
        loc->flags |= GPS_LOCATION_HAS_ACCURACY;

        int i;
        int temp_number;
        Charseg seg_satellite_using;

        memset(&g_satellites_info, 0, sizeof(UsingSatellitesInfo));//Added
        for (i=0; i<12; i++)
        {
            seg_satellite_using = wl_get_segments_by_index(info_segs, i+3);
            temp_number = str2int(seg_satellite_using.m_beg, seg_satellite_using.m_end);

            if (0 == temp_number)
            {
                break;
            }

            g_satellites_info.m_count++;
            g_satellites_info.m_number[i] = temp_number;
        }

        g_sv_status_flag |= 0x01;
    }
    else if (!memcmp(seg.m_beg, "GSV", 3))
    {
        Charseg seg_msg_index = wl_get_segments_by_index(info_segs, 2);
        volatile int msg_index = str2int(seg_msg_index.m_beg, seg_msg_index.m_end);
        Charseg seg_satellites_visble = wl_get_segments_by_index(info_segs, 3);
        volatile int satellites_visble = str2int(seg_satellites_visble.m_beg, seg_satellites_visble.m_end);
        GpsSvStatus *p_gps_sv_status = &g_sv_status_info;

        g_sv_status_flag |= (1<<msg_index);

        p_gps_sv_status->size = sizeof(GpsSvStatus);
        p_gps_sv_status->num_svs = satellites_visble;

        volatile int temp;
        if((satellites_visble / 4) < msg_index)
        {
            temp = satellites_visble % 4;
        }
        else
        {
            temp = 4;
        }

        LOGD("satellites visible = %d messages index = %d\n",satellites_visble,msg_index);
        
        int i;
        for(i = 0; i < temp;i++)
        {
            p_gps_sv_status->sv_list[i + 4 * (msg_index - 1)].size = sizeof(GpsSvInfo);
            
            Charseg seg_prn = wl_get_segments_by_index(info_segs, 4 + i * 4);
            int prn = str2int(seg_prn.m_beg,seg_prn.m_end);                
            p_gps_sv_status->sv_list[i + 4 * (msg_index - 1)].prn = prn;
            
            Charseg seg_elevation = wl_get_segments_by_index(info_segs,5 + i * 4);
            float elevation = str2float(seg_elevation.m_beg,seg_elevation.m_end);
            p_gps_sv_status->sv_list[i + 4 * (msg_index - 1)].elevation = elevation;
            
            Charseg seg_azimuth = wl_get_segments_by_index(info_segs,6 + i * 4);
            float azimuth = str2float(seg_azimuth.m_beg,seg_azimuth.m_end);
            p_gps_sv_status->sv_list[i + 4 * (msg_index - 1)].azimuth = azimuth;
            
            Charseg seg_snr = wl_get_segments_by_index(info_segs,7 + i * 4);
            float snr = str2float(seg_snr.m_beg,seg_snr.m_end);
            p_gps_sv_status->sv_list[ i + 4 * (msg_index - 1)].snr = snr;

            LOGD("prn:%d snr:%f elevation:%f azimuth:%f\n",prn,snr,elevation,azimuth);
        }
    }
    else if ( !memcmp(seg.m_beg, "VTG", 3))
    {
        Charseg seg_bearing = wl_get_segments_by_index(info_segs,1);
        Charseg seg_speed = wl_get_segments_by_index(info_segs,5);
        wl_get_bearing(loc, seg_bearing);
        wl_get_speed(loc, seg_speed);
    }
    else if ( !memcmp(seg.m_beg, "RMC", 3) ) 
    {
        Charseg seg_time = wl_get_segments_by_index(info_segs,1);
        Charseg seg_fixStatus = wl_get_segments_by_index(info_segs,2);
        Charseg seg_latitude = wl_get_segments_by_index(info_segs,3);
        Charseg seg_latitudeHemi = wl_get_segments_by_index(info_segs,4);
        Charseg seg_longitude = wl_get_segments_by_index(info_segs,5);
        Charseg seg_longitudeHemi = wl_get_segments_by_index(info_segs,6);
        Charseg seg_speed = wl_get_segments_by_index(info_segs,7);
        Charseg seg_bearing = wl_get_segments_by_index(info_segs,8);
        Charseg seg_date = wl_get_segments_by_index(info_segs,9);

        LOGD("fixStatus=%c", seg_fixStatus.m_beg[0]);
        if (seg_fixStatus.m_beg[0] == 'A')
        {
            wl_get_latlong(loc, seg_latitude, seg_latitudeHemi, seg_longitude, seg_longitudeHemi);
            wl_get_date(loc, seg_date, seg_time);
            wl_get_bearing(loc, seg_bearing);
            wl_get_speed  (loc, seg_speed);
        }
        {
            unsigned int i;
            g_sv_status_info.used_in_fix_mask &= 0x00;
            
            for(i = 0;i < g_satellites_info.m_count;i++)
            {
                if (g_satellites_info.m_number[i] <= 32)
                    g_sv_status_info.used_in_fix_mask |= (0x01 << (g_satellites_info.m_number[i] - 1)); 
            }
            
            g_sv_status_flag = 0;
            if (p_java_layer_callback->sv_status_cb)
                p_java_layer_callback->sv_status_cb(&g_sv_status_info);
            
            memset(&g_sv_status_info, 0, sizeof(GpsSvStatus));
            memset(&g_satellites_info, 0, sizeof(UsingSatellitesInfo));
        } 
    }

    if (loc->flags == 0x1f)
    {
        char   temp[256];
        char*  beg   = temp;
        char*  end = beg + sizeof(temp);
        struct tm   utc;

        beg += snprintf(beg, end-beg, "Location info:" );
        if (loc->flags & GPS_LOCATION_HAS_LAT_LONG) 
        {
            beg += snprintf(beg, end-beg, " lat=%g lon=%g", loc->latitude, loc->longitude);
        }
        if (loc->flags & GPS_LOCATION_HAS_ALTITUDE) 
        {
            beg += snprintf(beg, end-beg, " altitude=%g", loc->altitude);
        }
        if (loc->flags & GPS_LOCATION_HAS_SPEED) 
        {
            beg += snprintf(beg, end-beg, " speed=%g", loc->speed);
        }
        if (loc->flags & GPS_LOCATION_HAS_BEARING) 
        {
            beg += snprintf(beg, end-beg, " bearing=%g", loc->bearing);
        }
        if (loc->flags & GPS_LOCATION_HAS_ACCURACY) 
        {
            beg += snprintf(beg,end-beg, " accuracy=%g", loc->accuracy);
        }
        localtime_r( (time_t*) &loc->timestamp, &utc );
        beg += snprintf(beg, end-beg, " time=%s", asctime( &utc ) );
        LOGD("%s",temp);
        
        if (p_java_layer_callback->location_cb) 
        {
            p_java_layer_callback->location_cb(loc);
            loc->flags &= 0x0;
            loc->latitude = 0.0;
            loc->longitude = 0.0;
            loc->altitude = 0.0;
            loc->speed = 0.0;
            loc->bearing = 0.0;
            loc->accuracy = 0.0;
        }
        else 
        {
            LOGD("No callback function for report location");
        }
    }  
}

static void wl_report_cur_state(GpsStatusValue status)
{
    GpsStatus gps_stat;

    gps_stat.size = sizeof(GpsStatus);
    gps_stat.status = status;
    
    p_java_layer_callback->status_cb(&gps_stat);
    return;
}

static int wl_gps_xtra_init(GpsXtraCallbacks* callbacks) 
{
    LOGD("Enter ql_gps_xtra_init");
    p_java_layer_xtra_callbacks = callbacks;
    return 0;
}

static int wl_gps_xtra_inject_xtra_data(char* data, int length) 
{
    LOGD("Enter ql_gps_xtra_inject_xtra_data");
    return 0;
}

static int wl_gps_init_internal_process()
{
    g_need_reading_nmea = 1;
    g_read_port_thread = p_java_layer_callback->create_thread_cb( "wl_read_port_thread", wl_read_port_thread, NULL);

    return 0;
}

static int wl_send_at_cmd_internal(int cmd_index)
{
    char cmd_buf[32] = {0};
    int retry = 30;
    if (g_nmea_fd < 0)
    {
        return -1;
    }

    if (g_cur_atcmd != 0)
    {
        LOGD("AT port busy");
        return -2;
    }
    
    g_cur_atcmd = cmd_index;

    switch(cmd_index)
    {
    case ATCMD_ZGINIT:
        strcpy(cmd_buf, "AT+ZGINIT");
        break;
    case ATCMD_ZGMODE_3:
        strcpy(cmd_buf, "AT+ZGMODE=3");
        break;
    case ATCMD_ZGNMEA_31:
        strcpy(cmd_buf, "AT+ZGNMEA=31");
        break;
    case ATCMD_ZGNMEA_0:
        strcpy(cmd_buf, "AT+ZGNMEA=0");
        break;
    case ATCMD_ZGRUN_0:
        strcpy(cmd_buf, "AT+ZGRUN=0");
        break;
    case ATCMD_ZGRUN_2:
		strcpy(cmd_buf, "AT+ZGRUN=2");
		break;
	case ATCMD_ZGFIXRATE_65535:
		strcpy(cmd_buf, "AT+ZGFIXRATE=65535,0");
		break;	
	case ATCMD_ZGFIXRATE_1:
		strcpy(cmd_buf, "AT+ZGFIXRATE=1,0");
		break;
    default:
        LOGD("Unknown command");
        return -1;
        break;
    }

    strcat(cmd_buf, "\r");
    
    write(g_nmea_fd, cmd_buf, strlen(cmd_buf));

    while (retry-- > 0)
    {
        if (g_cur_atcmd == 0)
        {
            break;
        }

        usleep(200000);
    }

    if (g_cur_atcmd != 0)
    {
        LOGD("AT command error");
        g_cur_atcmd = 0;
        return -1;
    }

    return 0;
    
}

const GpsInterface* wl_get_gps_interface(struct gps_device_t* dev)
{
    LOGD("Enter wl_get_gps_interface");
    return &wl_GpsInterface;
}

static int wl_open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
    LOGD("Enter wl_open_gps");
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    //dev->common.close = NULL;
    dev->get_gps_interface = wl_get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}

static void wl_read_buffer_thread(void *param) 
{
	char info[LEN_GPS_INFO+1];
	int info_len = 0;
	char *temp = NULL;
	int err_flag = 0;
	GpsLocation loc;

	LOGD("[wl_read_buffer_thread]:ENTER.");

	memset(info,0,LEN_GPS_INFO+1);

	/*init location info*/
    memset(&loc, 0, sizeof(GpsLocation));
    loc.size = sizeof(GpsLocation);
	
readbuf:	
	pthread_mutex_lock(&mutex);
	LOGD("[wl_read_buffer_thread]:readbuf step1.");
	if(g_gps_info_buf->len == 0)
		goto wait;

	temp = strstr(g_gps_info_buf->m_buf,"\r\n");
	if(NULL == temp && g_gps_info_buf->len >= LEN_GPS_BUF)
	{
		LOGD("[wl_read_buffer_thread]:WRONG INFO 1.");
		err_flag = 1;
		memset(g_gps_info_buf,0,LEN_GPS_BUF);
	}
	else if(NULL == temp && g_gps_info_buf->len < LEN_GPS_BUF)
	{
		LOGD("[wl_read_buffer_thread]:THE INFO IS NOT COMPLETE!PLEASE WAIT...");
	}
	else if(temp != NULL && (temp - g_gps_info_buf->m_buf) > LEN_GPS_INFO)
	{
		LOGD("[wl_read_buffer_thread]:ERROR!THE INFO IS TOO LONG TO READ!");
		memmove(g_gps_info_buf->m_buf,temp+2,(LEN_GPS_BUF - (temp+2 - g_gps_info_buf->m_buf)));
		g_gps_info_buf->len -= (temp+2 - g_gps_info_buf->m_buf);
	}
	else if(temp != NULL && (temp - g_gps_info_buf->m_buf) <= LEN_GPS_INFO)
	{
		if(0 == err_flag)
		{
			LOGD("[wl_read_buffer_thread]:THE INFO IS COMPLETE.");
			memcpy(&info,g_gps_info_buf->m_buf,(temp - g_gps_info_buf->m_buf));
			info_len = temp - g_gps_info_buf->m_buf;
			info[info_len+1]="\0";
			memmove(g_gps_info_buf->m_buf,temp+2,(LEN_GPS_BUF - (temp+2 - g_gps_info_buf->m_buf)));
			g_gps_info_buf->len -= (temp+2 - g_gps_info_buf->m_buf);
		}	
		else
		{
			LOGD("[wl_read_buffer_thread]:WRONG INFO 2.");
			memmove(g_gps_info_buf->m_buf,temp+2,(LEN_GPS_BUF - (temp+2 - g_gps_info_buf->m_buf)));
			g_gps_info_buf->len -= (temp+2 - g_gps_info_buf->m_buf);
			err_flag = 0;
		}
	}
	
	pthread_mutex_unlock(&mutex);
	LOGD("[wl_read_buffer_thread]:readbuf step2.");
	temp = NULL;
	if(strlen(info) == 0)
	{
		LOGD("[wl_read_buffer_thread]:readbuf step3.");
		usleep(3000);
		goto readbuf;
	}

	LOGD("Get one line: %.*s", info_len, info);
	if ((g_cur_atcmd != 0) && (info_len >= 2) && (info[0] == 'O') && (info[1] == 'K'))
	{
		LOGD("wl_read_buffer_thread]:Find response of atcmd");
		g_cur_atcmd = 0;
	}
	else
	{
		LOGD("[wl_read_buffer_thread]:readbuf step4.");
		wl_parse_nmea_line(&loc, info, info_len);
	}
	memset(info,0,LEN_GPS_INFO+1);
	LOGD("[wl_read_buffer_thread]:readbuf step5.");
	goto readbuf;

wait:
	LOGD("[wl_read_buffer_thread]:readbuf step6.");
	pthread_mutex_unlock(&mutex);
	LOGD("[wl_read_buffer_thread]:readbuf step7.");
	usleep(3000);
	LOGD("[wl_read_buffer_thread]:readbuf step8.");
	goto readbuf;
}

static int wl_get_nmea_port(char *port)
{
	FILE * fp = NULL;
	char *temp = NULL;
	char str_buf[128]={0};
	
	fp = fopen(NMEA_PORT_PATH_CONFIG,"r");
	if(NULL == fp)
		return -1;
	
	while(fgets(str_buf,128,fp))
	{
		temp = strstr(str_buf,"NMEA_PORT");
		if(NULL != temp)
		{
			strncpy(port,temp+10,20);
			break;
		}
	}

	if(strlen(port) == 0)
		return -1;
	
	fclose(fp);
	fp = NULL;
	return 0;
}

static void wl_read_port_thread(void *param) 
{
	gps_info_buf gps_buf;
    int read_len = 0;
    char *read_buf = NULL;
    int read_buf_len = 0;
    char *line_buf = NULL;
    int line_len = 0;
    int sel_count = 0;
	int try_count = 0;
	char nmea_port[20] = {0};
  //  struct timeval sel_timeout = {2,0};
  	
	struct timeval sel_timeout;	
	sel_timeout.tv_sec = 1;	
	sel_timeout.tv_usec = 0;	 

	LOGD("[wl_read_port_thread]:ENTER.");

    /*init information of utc time*/
    g_utc_info.m_year = -1;
    g_utc_info.m_month = -1;
    g_utc_info.m_day = -1;
    g_utc_info.m_sub = wl_calc_utc_sub();

	/*init gps_buf*/
	g_gps_info_buf = &gps_buf;
	memset(g_gps_info_buf,0,sizeof(gps_buf));

	/*get port*/
	if(wl_get_nmea_port(nmea_port)<0)
	{	
		LOGD("[wl_read_port_thread]:Read Port Config Wrong!");
		goto cleanup;
	}
    
Open_port:
	LOGD("[wl_read_port_thread]:open port.");
    while (g_need_reading_nmea)
    {
        g_nmea_fd = open(nmea_port, O_RDWR);
        if (g_nmea_fd > 0) 
        {
            struct termios ios;
            memset(&ios, 0, sizeof(ios));
            tcgetattr( g_nmea_fd, &ios);
            cfmakeraw(&ios);
            ios.c_lflag = 0; 
            cfsetispeed(&ios, 115200);
            cfsetospeed(&ios, 115200);
            tcsetattr( g_nmea_fd, TCSANOW, &ios );
			LOGD("[wl_read_port_thread]:open port successfully.");
            break;
        }
        else
        {
            LOGD("[wl_read_port_thread]:Error! Can not open NMEA port, will try later,s%",nmea_port);
            sleep(2);
            continue;
        }
    }

	g_read_buff_thread = p_java_layer_callback->create_thread_cb( "wl_read_buffer_thread", wl_read_buffer_thread, NULL);

	fd_set nmea_port_set;
	
Readloop:
	LOGD("[wl_read_port_thread]:read loop.");
    while (g_need_reading_nmea)
    {	
    	usleep(3000);
    	LOGD("[wl_read_port_thread]:read loop step1");
		FD_ZERO(&nmea_port_set);
		FD_SET(g_nmea_fd, &nmea_port_set);
    	sel_count = select((g_nmea_fd+1), &nmea_port_set, NULL, NULL, &sel_timeout);
		LOGD("[wl_read_port_thread]:read loop step2");
        if (0 == sel_count)
        {	
        	LOGD("[wl_read_port_thread]:read loop step3");
            continue;
        }
        else if (sel_count < 0)
        {
        	if(try_count <= 3)
        	{
				LOGD("[wl_read_port_thread]:Reopen GPS port.");
				try_count++;
				goto Open_port;
        	}
			else
			{
				LOGD("[wl_read_port_thread]:Error!exit GPS process");
				goto cleanup;
			}
        }

		pthread_mutex_lock(&mutex);
		LOGD("[wl_read_port_thread]:read loop step4");
		read_buf = &(g_gps_info_buf->m_buf[g_gps_info_buf->len]);
		read_buf_len = LEN_GPS_BUF - g_gps_info_buf->len;
		
        if (0 == read_buf_len)
        {
            LOGD("[wl_read_port_thread]:Buffer full, please wait..\n");
			goto wait;
        }

		LOGD("[wl_read_port_thread]:read loop step5");
        read_len = read(g_nmea_fd, read_buf, read_buf_len);
		
		if(read_len)
		{
			LOGD("[wl_read_port_thread]:read data.");
			g_gps_info_buf->len += read_len;
			g_gps_info_buf->m_buf[g_gps_info_buf->len+1] = "\0";
		}
		
		pthread_mutex_unlock(&mutex);

		LOGD("[wl_read_port_thread]:read loop step6");
        if (read_len <= 0)
        {
            usleep(3000);
            if(try_count <= 3)
        	{
				LOGD("[wl_read_port_thread]:Reopen GPS port.");
				try_count++;
				goto Open_port;
        	}
			else
           	{
				LOGD("[wl_read_port_thread]:Error!exit GPS process");
				goto cleanup;
			}
        }

		try_count = 0;
		LOGD("[wl_read_port_thread]:read loop step7");
    }
	
cleanup:
	LOGD("[wl_read_port_thread]:clean up.");
    if (g_nmea_fd > 0)
    {
        close(g_nmea_fd);
        g_nmea_fd = -1;
    }
	
    wl_gps_cleanup();
	g_gps_info_buf = NULL;
	return NULL;

wait:
	LOGD("[wl_read_port_thread]:wait...");
	pthread_mutex_unlock(&mutex);
	LOGD("[wl_read_port_thread]:read loop step8");
	usleep(3000);
	LOGD("[wl_read_port_thread]:read loop step9");
	goto Readloop;
}

//interfaces
static int wl_gps_init(GpsCallbacks* callbacks) 
{
    int ret;
    GpsStatus gps_stat;
    
    LOGD("This is Welink GPS module, VERSION: %s",DRIVER_VERSION);

    if (NULL == callbacks)
    {
        return -1;
    }

    p_java_layer_callback = callbacks;

    if (g_is_internal_initialized == 0)
    {
        ret = wl_gps_init_internal_process();
        if (ret < 0)
        {
            LOGE("Error while initialize GPS device");
            return -1;
        }
    }

    g_is_internal_initialized = 1;
    g_cur_gps_status = GPS_STATUS_ENGINE_ON;
    wl_report_cur_state(g_cur_gps_status);
    return 0;
}

static void wl_gps_cleanup(void) 
{
    int retry = 15;

    LOGD("Enter wl_gps_cleanup");    
    g_need_reading_nmea = 0;

    while (retry-- > 0)
    {
        if (g_nmea_fd <= 0)
        {
            break;
        }

        usleep(200000);
    }

    if (g_nmea_fd > 0)
    {
        close(g_nmea_fd);
        g_nmea_fd = -1;
    }

    g_is_internal_initialized = 0;

    memset(&g_satellites_info, 0, sizeof(UsingSatellitesInfo));
    g_sv_status_flag = 0;
    memset(&g_sv_status_info, 0, sizeof(GpsSvStatus));

    g_cur_atcmd = 0;

    g_cur_gps_status = GPS_STATUS_NONE;
    wl_report_cur_state(g_cur_gps_status);
}

static int wl_gps_start(void) 
{
    int ret;
    
    LOGD("Enter wl_gps_start");

    if ((g_cur_gps_status == GPS_STATUS_NONE)
        || (g_cur_gps_status == GPS_STATUS_ENGINE_OFF))
    {
        ret = wl_gps_init(p_java_layer_callback);
        if (ret != 0) return -1;
    }

    if (g_is_internal_initialized == 0)
    {
        return -1;
    }

    ret = wl_send_at_cmd_internal(ATCMD_ZGINIT);
    if (ret < 0)
    {
        return -1;
    }

    ret = wl_send_at_cmd_internal(ATCMD_ZGMODE_3);
    if (ret < 0)
    {
        return -1;
    }

	ret = wl_send_at_cmd_internal(ATCMD_ZGFIXRATE_65535);
    if (ret < 0)
    {
        return -1;
    }

	ret = wl_send_at_cmd_internal(ATCMD_ZGNMEA_31);
    if (ret < 0)
    {
        return -1;
    }
	
    ret = wl_send_at_cmd_internal(ATCMD_ZGRUN_2);
    if (ret < 0)
    {
        return -1;
    }

    g_cur_gps_status = GPS_STATUS_SESSION_BEGIN;
    wl_report_cur_state(g_cur_gps_status);
    return 0;    
}

static int wl_gps_stop(void) 
{
    int ret;
    
    LOGD("Enter wl_gps_stop");

    if (g_is_internal_initialized == 0)
    {
        LOGD("Not init, return directly");
        return -1;
    }

    ret = wl_send_at_cmd_internal(ATCMD_ZGRUN_0);
    if (ret < 0)
    {
        return -1;
    }

    ret = wl_send_at_cmd_internal(ATCMD_ZGNMEA_0);
    if (ret < 0)
    {
        return -1;
    }

	ret = wl_send_at_cmd_internal(ATCMD_ZGFIXRATE_1);
    if (ret < 0)
    {
        return -1;
    }
    
    g_cur_gps_status = GPS_STATUS_SESSION_END;
    wl_report_cur_state(g_cur_gps_status);
    return 0;
}

static int wl_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    LOGD("wl_gps_inject_time: Not support");
    return 0;
}

static int wl_gps_inject_location(double latitude, double longitude, float accuracy)
{
    LOGD("ql_gps_inject_location: Not support");
    return 0;
}

static void wl_gps_delete_aiding_data(GpsAidingData flags) 
{
    LOGD("wl_gps_delete_aiding_data: Not support");
}


static int wl_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
            uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    LOGD("ql_gps_set_position_mode: Not support");
    return 0;
}

static const void* wl_gps_get_extension(const char* name) 
{
    LOGD("Enter wl_gps_get_extension: para=%s", name);
    if (!strcmp(name, GPS_XTRA_INTERFACE))
        return &wl_GpsXtraInterface;
    return NULL;
}

