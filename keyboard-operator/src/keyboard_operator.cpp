#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <array>

void changemode(int);
int  kbhit(void);
int main(void)
{
    std::array<bool, 256> a = std::array<bool, 256>();
    int ch;
    changemode(1);
    while (true) {
        // Waiting for some keyboard input.
        while (!kbhit()) {
            for (int i =0; i < 256; ++i){
                if (a[i]){
                    a[i] = false;
                    printf("Unpressed %c\n", i);
                }
            }
        }

        // something has been detected. now get that.
        ch = getchar();

        if(!a[ch]){
            a[ch] = true;
            printf("Pressed %c\n", ch);
        }


    }
    changemode(0);
    return 0;
}

void changemode(int dir)
{
    static struct termios oldt, newt;

    if ( dir == 1 )
    {
        tcgetattr( STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);

}