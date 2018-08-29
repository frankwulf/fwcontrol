// Author:  Frank Wulf
// Version: 2.0 (2018-08-24)
//
// This program monitors temperatures of both system and hard drives and
// changes fan speeds accordingly.
//
// Copyright (C) 2017 Frank Wulf
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <scsi/sg.h>

#define BUFSIZE 256
#define FAN_MODE 1
#define MAX_STEP 10

#define SMART_BUFFER_SIZE 512
#define SMART_SENSE_BUFFER_SIZE 32
#define SMART_CMD_LENGTH 12

enum {
    ATA_OP_CHECKPOWERMODE1 = 0xe5,
    ATA_OP_CHECKPOWERMODE2 = 0x98,
    ATA_USING_LBA = (1 << 6),
    ATA_STAT_DRQ = (1 << 3),
    ATA_STAT_ERR = (1 << 0),
};

enum {
    SG_ATA_16 = 0x85,
    SG_ATA_16_LEN = 16,
    SG_ATA_PROTO_NON_DATA = (3 << 1)
};

enum {SG_CDB2_CHECK_COND = (1 << 5)};

enum {SYS, HDD};

struct s_fan {
    char name[20];
    char control[2];
    char pwm_enable[50];
    char pwm_write[50];
    short stop_delay;
    short decr_delay;
    short interval[2];
    char interpolate[2];
    short hyst[2];
    short count_scan[2];
    short count_step[2];
    time_t next_check[2];
    char loglevel;
    short temp[2];
    unsigned char pwm[2];
    unsigned char actual_pwm;
    unsigned char idle_pwm;
    unsigned char error_pwm[2];
    time_t min_decr_time;
    time_t min_stop_time;
} *fan;

struct s_temp_pwm {
    short temp;
    unsigned char pwm;
} ***data_pwm;

struct s_temp {
    char name[50];
    short temp;
} *temp_buf;

char ***scan_hdd, ***scan_sys;
char first_check = 1;

short count_fans, count_total[2], max_per_fan[2];
time_t now, next_check;

typedef void (*sighandler_t)(int);

static sighandler_t handle_signal(int sig_nr, sighandler_t signalhandler) {
    struct sigaction new_sig, old_sig;
    new_sig.sa_handler = signalhandler;
    sigemptyset(&new_sig.sa_mask);
    new_sig.sa_flags = SA_RESTART;
    if (sigaction(sig_nr, &new_sig, &old_sig) < 0)
        return SIG_ERR;
    return old_sig.sa_handler;
}

static void start_daemon(const char *log_name, int facility) {
    int i;
    pid_t pid;
    // Fork off parent process
    if ((pid = fork()) != 0)
        exit(EXIT_FAILURE);
    // Create new SID for child process
    if (setsid() < 0)
        exit(EXIT_FAILURE);
    // Ignore signal SIGHUP
    handle_signal(SIGHUP, SIG_IGN);
    // Fork off child process
    if ((pid = fork()) != 0)
        exit(EXIT_FAILURE);
    // Change working directory
    chdir("/");
    // Change file mode mask
    umask(0);
    // Close all file descriptors
    for (i = sysconf(_SC_OPEN_MAX); i > 0; i--)
        close(i);
    openlog(log_name, LOG_PID | LOG_CONS | LOG_NDELAY, facility);
}

static inline int write_fan(char *name, unsigned char value) {
    FILE *fp;

    if ((fp = fopen(name, "w")) == NULL) {
        syslog(LOG_ERR, "Error opening file %s!\n", name);
        return -1;
    }
    fprintf(fp, "%d", value);
    fclose(fp);

    return 0;
}

int read_mem_conf(void) {
    FILE *fp;
    size_t len;
    char *name, *value, *value_w_comma, *total[2] = { NULL }, buf[BUFSIZE];
    int  per_fan[2], t;

    if ((fp = fopen("/etc/fwcontrol.conf", "r")) == NULL) {
        syslog(LOG_ERR, "Error opening configuration file\n");
        return -1;
    }

    while (fgets(buf, BUFSIZE, fp) != NULL) {
        if (buf[0] == '[') {
            ++count_fans;
            per_fan[SYS] = 0;
            per_fan[HDD] = 0;
        } else {
            name = strtok(buf, "=");
            value = strtok(NULL, "=");
            value = strtok(value, "\n");
            if (strcmp(name, "sys_input") == 0 ||
               strcmp(name, "scan_hdd") == 0) {
                t = strstr(name, "sys") ? SYS : HDD;
                value = strtok(value, ",");
                while (value != NULL) {
                    value_w_comma = malloc(strlen(value) + 2);
                    strcpy(value_w_comma, value);
                    strcat(value_w_comma, ",");
                    if (count_total[t] == 0) {
                        total[t] = calloc(1, strlen(value_w_comma) + 1);
                        strcat(total[t], value_w_comma);
                        ++count_total[t];
                    } else if (strstr(total[t], value_w_comma) == NULL) {
                        len = strlen(total[t]) + strlen(value_w_comma) + 1;
                        total[t] = realloc(total[t], len);
                        strcat(total[t], value_w_comma);
                        ++count_total[t];
                    }
                    free(value_w_comma);
                    if (++per_fan[t] > max_per_fan[t])
                        ++max_per_fan[t];
                    value = strtok(NULL, ",");
                }
            }
        }
    }
    fclose(fp);

    syslog(LOG_NOTICE,
      "Controlling %d %s by monitoring %d system %s and %d %s",
      count_fans, (count_fans == 1) ? "fan" : "fans", count_total[SYS],
      (count_total[SYS] == 1) ? "sensor" : "sensors", count_total[HDD],
      (count_total[HDD] == 1) ? "hard drive" : "hard drives");
    free(total[SYS]);
    free(total[HDD]);

    return 0;
}

int read_fan_conf(void) {
    FILE *fp;
    char *name, *value, buf[BUFSIZE];
    short i = -1, j, k[2], t;

    now = time(NULL);
    if ((fp = fopen("/etc/fwcontrol.conf", "r")) == NULL) {
        syslog(LOG_ERR, "Error opening configuration file\n");
        return -1;
    }

    while (fgets(buf, BUFSIZE, fp) != NULL) {
        if (buf[0] == '[') {
            strcpy(fan[++i].name, strtok(buf, "[]\n"));
            k[SYS] = 0;
            k[HDD] = 0;
        } else if (i >= 0) {
            name = strtok(buf, "=");
            value = strtok(NULL, "=");
            value = strtok(value, "\n");
            t = strstr(name, "sys") ? SYS : HDD;
            if (strncmp(name, "control_by_", 11) == 0) {
                fan[i].control[t] = atoi(value);
                fan[i].next_check[t] = now;
                fan[i].error_pwm[t] = 255;
            } else if (strcmp(name, "pwm_enable") == 0) {
                strcpy(fan[i].pwm_enable, value);
                write_fan(fan[i].pwm_enable, FAN_MODE);
            } else if (strcmp(name, "pwm_write") == 0) {
                strcpy(fan[i].pwm_write, value);
            } else if (strcmp(name, "stop_delay") == 0) {
                fan[i].stop_delay = atoi(value);
            } else if (strcmp(name, "decrease_delay") == 0) {
                fan[i].decr_delay = atoi(value);
            } else if (strcmp(name, "loglevel") == 0) {
                fan[i].loglevel = atoi(value);
            } else if (strncmp(name, "interval_", 9) == 0) {
                fan[i].interval[t] = atoi(value);
            } else if (strncmp(name, "interpolate_", 12) == 0) {
                fan[i].interpolate[t] = atoi(value);
            } else if (strncmp(name, "hyst_", 5) == 0) {
                fan[i].hyst[t] = atoi(value);
            } else if (strcmp(name, "idle_pwm") == 0) {
                fan[i].idle_pwm = atoi(value);
            } else if (strncmp(name, "error_pwm_", 10) == 0) {
                fan[i].error_pwm[t] = atoi(value);
            } else if (strcmp(name, "sys_input") == 0 ||
                       strcmp(name, "scan_hdd") == 0 ) {
                value = strtok(value, ",");
                while (value != NULL) {
                    if (t == SYS)
                        strcpy(scan_sys[i][k[t]++], value);
                    else
                        strcpy(scan_hdd[i][k[t]++], value);
                    value = strtok(NULL, ",");
                }
                fan[i].count_scan[t] = k[t];
            } else if (strncmp(name, "temp_pwm_", 9) == 0) {
                j = 0;
                value = strtok(value, ",");
                while (value != NULL && ++j <= MAX_STEP) {
                    data_pwm[t][i][j-1].temp = atoi(value);
                    value = strtok(NULL, ",");
                    data_pwm[t][i][j-1].pwm = atoi(value);
                    value = strtok(NULL, ",");
                }
                fan[i].count_step[t] = j;
            }
        }
    }
    fclose(fp);

    return 0;
}

static inline int sgio_send(int fd, unsigned char cmd, unsigned char *rv) {
    unsigned char cdb[SG_ATA_16_LEN], sb[32], *desc, status, error;
    sg_io_hdr_t io_hdr;

    memset(&cdb, 0, sizeof(cdb));
    memset(&sb, 0, sizeof(sb));
    memset(&io_hdr, 0, sizeof(io_hdr));

    cdb[0] = SG_ATA_16;
    cdb[1] = SG_ATA_PROTO_NON_DATA;
    cdb[2] = SG_CDB2_CHECK_COND;
    cdb[13] = ATA_USING_LBA;
    cdb[14] = cmd;

    io_hdr.cmd_len = SG_ATA_16_LEN;
    io_hdr.interface_id = 'S';
    io_hdr.mx_sb_len = sizeof(sb);
    io_hdr.dxfer_direction = SG_DXFER_NONE;
    io_hdr.dxfer_len = 0;
    io_hdr.dxferp = NULL;
    io_hdr.cmdp = cdb;
    io_hdr.sbp = sb;
    io_hdr.pack_id = 0;
    io_hdr.timeout = 500; // Milliseconds

    if (ioctl(fd, SG_IO, &io_hdr) == -1) {
        syslog(LOG_ERR, "ioctl() failed (cmd %u, %s)\n", cmd, strerror(errno));
        return -1;
    }

    desc = sb + 8;
    status = desc[13];
    error = desc[3];
    if (rv)
        *rv = desc[5];

    if (status & (ATA_STAT_ERR | ATA_STAT_DRQ)) {
        syslog(LOG_ERR, "SG_IO command %u failed (status %u, error %u)\n",
          cmd, status, error);
        return -1;
    }

    return 0;
}

static inline char get_hdd_status(char *name) {
    int fd, ret;
    unsigned char state;

    if ((fd = open(name, O_RDONLY)) < 0) {
        syslog(LOG_ERR, "Error opening file %s!\n", name);
        return -1;
    }

    ret = 1;
    if (sgio_send(fd, ATA_OP_CHECKPOWERMODE1, &state) &&
        sgio_send(fd, ATA_OP_CHECKPOWERMODE2, &state))
        ret = 0;
    close(fd);

    return (state == 0) ? 0 : 1; // 0 = Sleeping, 1 = Running
}

static inline short get_hdd_temp(char *name) {
    short hdd_temp;
    int fd;
    unsigned char buffer[SMART_BUFFER_SIZE] = "";
    unsigned char sense_buffer[SMART_SENSE_BUFFER_SIZE];
    unsigned char smart_read_cdb[SMART_CMD_LENGTH] =
                  {0xa1, 0x0c, 0x0e, 0xd0, 1, 0, 0x4f, 0xc2, 0, 0xb0, 0, 0};

    sg_io_hdr_t io_hdr;

    if ((fd = open(name, O_RDONLY)) < 0) {
        syslog(LOG_ERR, "Error opening file %s!\n", name);
        return -1;
    }

    memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
    io_hdr.interface_id = 'S';
    io_hdr.cmd_len = SMART_CMD_LENGTH;
    io_hdr.mx_sb_len = SMART_SENSE_BUFFER_SIZE;
    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    io_hdr.dxfer_len = SMART_BUFFER_SIZE;
    io_hdr.dxferp = buffer;
    io_hdr.cmdp = smart_read_cdb;
    io_hdr.sbp = sense_buffer;
    io_hdr.timeout = 500; // Milliseconds

    if (ioctl(fd, SG_IO, &io_hdr) < 0) {
        syslog(LOG_ERR, "ioctl() call for reading temperature failed\n");
        close(fd);
        return -1;
    }

    for (register int i = 2; i < 361; i += 12)
        if ((int)buffer[i] == 194) {
            hdd_temp = ((long long int)((buffer[i+5])|
                       (buffer[i+6]<<8)|
                       (buffer[i+7]<<16)|
                       (buffer[i+8]<<24)|
                       ((long long int)buffer[i+9]<<32)|
                       ((long long int)buffer[i+10]<<40))) & 0xFF;
            break;
        }
    close(fd);

    return hdd_temp;
}

static inline short get_sys_temp(char *name) {
    FILE *fp;
    char buf[BUFSIZE];
    short sys_temp;

    if ((fp = fopen(name, "r")) == NULL) {
        syslog(LOG_ERR, "Error opening file %s!\n", name);
        return -1;
    }
    if (fgets(buf, BUFSIZE, fp) != NULL)
        sys_temp = atoi(buf) / 1000;
    fclose(fp);

    return sys_temp;
}

static inline unsigned char calc_pwm(char t, int i, short temp) {
    unsigned char pwm = 0;
    short j = 0;

    while (temp >= data_pwm[t][i][j].temp) {
        pwm = data_pwm[t][i][j].pwm;
        if (++j >= fan[i].count_step[t])
            break;
    }

    if (fan[i].interpolate[t] == 1 && j > 0 && j < fan[i].count_step[t])
        pwm = ((float)data_pwm[t][i][j].pwm - data_pwm[t][i][j-1].pwm) /
              (data_pwm[t][i][j].temp - data_pwm[t][i][j-1].temp) *
              (temp - data_pwm[t][i][j-1].temp) + data_pwm[t][i][j-1].pwm + 0.5;

    return pwm;
}

static inline int control_fan_speed(void) {
    struct stat device;
    unsigned char new_pwm;
    char sw_buffered, sw_checked, sw_error;
    short temp, dev_temp, count_buffer = 0;

    now = time(NULL);
    next_check = 0;
    for (int i = 0; i < count_fans; i++) {
        sw_checked = 0;
        for (int type = 0; type < 2; type++) {
            if (fan[i].control[type] == 1 && now >= fan[i].next_check[type]) {
                temp = 0;
                sw_error = 0;
                for (int j = 0; j < fan[i].count_scan[type]; j++) {
                    dev_temp = 0;

                    // Check if temperature is in buffer
                    sw_buffered = 0;
                    for (int k = 0; k < count_buffer; k++)
                        if ((type == SYS && strcmp(temp_buf[k].name,
                           scan_sys[i][j]) == 0) || (type == HDD &&
                           strcmp(temp_buf[k].name, scan_hdd[i][j]) == 0)) {
                            dev_temp = temp_buf[k].temp;
                            sw_buffered = 1;
                            break;
                        }

                    if (sw_buffered == 0) {
                        if (type == HDD && stat(scan_hdd[i][j], &device) == 0 &&
                           (device.st_mode & S_IFMT) == S_IFBLK &&
                           get_hdd_status(scan_hdd[i][j]) == 1)
                            // Get hard drive temperature
                            dev_temp = get_hdd_temp(scan_hdd[i][j]);
                        else if (type == SYS)
                            // Get system temperature
                            dev_temp = get_sys_temp(scan_sys[i][j]);

                        if (dev_temp < 0)
                            sw_error = 1;

                        // Write temperature to buffer
                        if (type == SYS)
                            strcpy(temp_buf[count_buffer].name, scan_sys[i][j]);
                        else
                            strcpy(temp_buf[count_buffer].name, scan_hdd[i][j]);

                        temp_buf[count_buffer++].temp = dev_temp;
                    }

                    if (dev_temp > temp)
                        temp = dev_temp;
                }

                if (sw_error == 1 && calc_pwm(type, i, temp) <
                   fan[i].error_pwm[type])
                    fan[i].pwm[type] = fan[i].error_pwm[type];
                else if (!(temp < fan[i].temp[type] && (fan[i].hyst[type] >
                   fan[i].temp[type] - temp || now < fan[i].min_decr_time)) &&
                   (fan[i].temp[type] != temp || first_check == 1)) {
                    fan[i].pwm[type] = calc_pwm(type, i, temp);
                    fan[i].temp[type] = temp;
                }

                while ((fan[i].next_check[type] += fan[i].interval[type]) <=
                   now);
                sw_checked = 1;
            }

            if (fan[i].control[type] == 1 && (next_check == 0 ||
               next_check > fan[i].next_check[type]))
                next_check = fan[i].next_check[type];
        }

        if (sw_checked == 0)
            continue;

        if (fan[i].pwm[SYS] > fan[i].pwm[HDD])
            new_pwm = fan[i].pwm[SYS];
        else
            new_pwm = fan[i].pwm[HDD];

        if (new_pwm == fan[i].actual_pwm && first_check != 1)
            continue;
        else if (new_pwm < fan[i].actual_pwm)
            // Check if decrease delay time has passed
            if (now < fan[i].min_decr_time)
                continue;
            else if (new_pwm == 0)
                //  Check if stop delay time has passed
                if (now < fan[i].min_stop_time)
                    if (fan[i].idle_pwm != 0 && fan[i].idle_pwm !=
                       fan[i].actual_pwm)
                        // Set fan speed to idle
                        new_pwm = fan[i].idle_pwm;
                    else
                        continue;

        // Set new fan speed
        write_fan(fan[i].pwm_write, new_pwm);
        if (fan[i].loglevel >= 1)
            if (first_check == 1)
                syslog(LOG_NOTICE,
                  "%s: PWM set to %d (SYS: %d°%s, HDD: %d°%s)\n",
                  fan[i].name, new_pwm, fan[i].temp[SYS],
                  (fan[i].pwm[SYS] < new_pwm) ? "C" : "C*", fan[i].temp[HDD],
                  (fan[i].pwm[HDD] < new_pwm) ? "C" : "C*");
            else
                syslog(LOG_NOTICE,
                  "%s: PWM changed from %d to %d (SYS: %d°%s, HDD: %d°%s)\n",
                  fan[i].name, fan[i].actual_pwm, new_pwm, fan[i].temp[SYS],
                  (fan[i].pwm[SYS] < new_pwm) ? "C" : "C*", fan[i].temp[HDD],
                  (fan[i].pwm[HDD] < new_pwm) ? "C" : "C*");
        if (fan[i].actual_pwm < new_pwm)
            fan[i].min_decr_time = now + fan[i].decr_delay;
        if (fan[i].actual_pwm == 0 && new_pwm != 0)
            fan[i].min_stop_time = now + fan[i].stop_delay;
        fan[i].actual_pwm = new_pwm;
    }
    if (first_check == 1)
        first_check = 0;

    return 0;
}

int main (int argc, char **argv) {
    int h, r;
    start_daemon ("fwcontrol", LOG_LOCAL0);
    syslog(LOG_NOTICE, "fwcontrol started ...\n");

    // Determine number of configured fans, system sensors and hard drives and
    // allocate memory accordingly
    read_mem_conf();

    fan = calloc(count_fans, sizeof(struct s_fan));
    temp_buf = calloc(count_total[SYS]+count_total[HDD], sizeof(struct s_temp));
    data_pwm = calloc(2, sizeof(struct s_temp_pwm));
    for (h = 0; h < 2; h++) {
        data_pwm[h] =  calloc(count_fans, sizeof(data_pwm[0]));
        for (r = 0; r < count_fans; r++)
            data_pwm[h][r] = calloc(MAX_STEP, sizeof(data_pwm[0][0]));
    }
    scan_sys = calloc(count_fans, sizeof(char**));
    scan_hdd = calloc(count_fans, sizeof(char**));
    for (h = 0; h < count_fans; h++) {
        scan_sys[h] = calloc(max_per_fan[SYS], sizeof(char*));
        scan_hdd[h] = calloc(max_per_fan[HDD], sizeof(char*));
        for (r = 0; r < max_per_fan[SYS]; r++)
            scan_sys[h][r] = calloc(50, sizeof(char));
        for (r = 0; r < max_per_fan[HDD]; r++)
            scan_hdd[h][r] = calloc(10, sizeof(char));
    }

    // Read fan configuration
    read_fan_conf();
    while (1) {
        control_fan_speed();
        sleep(next_check - now);
    }
    syslog(LOG_NOTICE, "fwcontrol terminated.\n");
    closelog();
    return EXIT_SUCCESS;
}
