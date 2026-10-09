#pragma once

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#define VIDEO_WIDTH 720
#define VIDEO_HEIGHT 480
#define VIDEO_FPS 60

/* Each reader holds at most one frame at a time. */
typedef enum { VIDEO_READER_UVC, VIDEO_READER_CONSOLE, VIDEO_READERS } video_reader_t;

typedef struct {
    const uint8_t *jpeg;
    size_t len;
    uint32_t seq;   /* 1 for the first frame since boot */
    int64_t t_us;   /* esp_timer time the frame was captured */
} video_frame_t;

void video_init(void);
/* Waits for a frame newer than the last one this reader took. Valid until video_release. */
bool video_next(video_reader_t who, video_frame_t *f, TickType_t wait);
/* The newest frame, even if this reader already took it. */
bool video_newest(video_reader_t who, video_frame_t *f);
void video_release(video_reader_t who);
/* A 720x480 YUV 4:2:2 frame (each pixel pair V Y0 U Y1 in memory) for the NTSC decoder to fill; video_field_done hands it to the encoder
 * and the next call returns the other one. The test pattern runs whenever no field has arrived for 100 ms. */
uint8_t *video_field_buffer(void);
void video_field_done(void);
void video_command(int argc, char **argv);
