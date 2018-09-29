#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv)
{
    unsigned char* rom = (unsigned char *)malloc(32768);

    memset(rom, 0, 32768);

    fread(rom, 1, 32768, stdin);

    unsigned char img[512][512][3];

    for(int bit_address = 0; bit_address < 32768 * 8; bit_address++) {
        int value = rom[bit_address / 8] & (128 >> bit_address % 8);
        int xpixel = bit_address % 8;
        int ypixel = (bit_address / 8) % 8;
        int xtile = (bit_address / 64) % 64;
        int ytile = (bit_address / (64 * 64));
        int x = xtile * 8 + xpixel;
        int y = ytile * 8 + ypixel;
        fprintf(stderr, "%d %d %d\n", bit_address, x, y);
        unsigned char *color = img[y][x];
        color[0] = value ? 0xff : 0x00;
        color[1] = value ? 0xff : 0x00;
        color[2] = value ? 0xff : 0x00;
    }
        
    printf("P6 512 512 255\n");
    fwrite(img, 512, 512 * 3, stdout);
}
