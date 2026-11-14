/* 帧打包的离线对拍测试。生成一帧写到文件，再用 tools/eic_frame.py 解析验证。
 *
 * 编译： gcc -O2 -o /tmp/test_frame tests/test_frame.c src/lib/frame_codec.c -lm
 * 运行： /tmp/test_frame /tmp/frame.bin
 * 校验： python3 tools/eic_frame.py --dump /tmp/frame.bin
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lib/frame_codec.h"

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "/tmp/frame.bin";

    fusion_frame_t f;
    memset(&f, 0, sizeof(f));

    f.ts_ms = 123456;

    /* 手掌绕 z 转 90 度 */
    const float half = (float)(M_PI / 4.0);
    f.q_hand[0] = cosf(half);
    f.q_hand[1] = 0.0f;
    f.q_hand[2] = 0.0f;
    f.q_hand[3] = sinf(half);

    /* 十根手指各自给一个已知的小角度 */
    for (int i = 0; i < EI_FINGER_COUNT; i++) {
        const float a = (float)(M_PI / 180.0) * (float)(i + 1);
        f.q_rel[i][0] = cosf(a * 0.5f);
        f.q_rel[i][1] = sinf(a * 0.5f);
        f.q_rel[i][2] = 0.0f;
        f.q_rel[i][3] = 0.0f;
    }

    for (int i = 0; i < EI_IMU_COUNT; i++) {
        f.temp[i] = 25.0f + (float)i * 0.5f;
    }

    uint8_t buf[EI_FRAME_SIZE];
    const size_t n = frame_build(buf, sizeof(buf), &f, 65533);
    if (n != EI_FRAME_SIZE) {
        printf("打包失败，n=%zu\n", n);
        return 1;
    }

    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        printf("写文件失败：%s\n", path);
        return 1;
    }
    fwrite(buf, 1, n, fp);
    fclose(fp);

    printf("已写入 %s，%zu 字节，CRC=0x%04X\n", path, n, frame_crc16(buf, n - 2));
    printf("手掌 q=(%.4f %.4f %.4f %.4f)\n", f.q_hand[0], f.q_hand[1], f.q_hand[2],
           f.q_hand[3]);
    return 0;
}
