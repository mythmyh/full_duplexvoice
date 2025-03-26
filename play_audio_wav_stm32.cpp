#include <iostream>
#include <map>
#include <iostream>
#include <fstream>
#include <WinSock2.h>
#include <ws2tcpip.h>
#ifdef __cplusplus
extern "C"
{
#endif

#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include <libswscale/swscale.h>

#include <libswresample/swresample.h>

#ifdef __cplusplus
}
#endif

#pragma comment(lib, "Ws2_32.lib")
#define PORT 12345
#define SDL_MAIN_HANDLED
#include <iostream>
#include <fstream>
#include <vector>
#include <iostream>
#include "SDL2/SDL.h"
using namespace std;
#define SDL_AUDIO_BUFFER_SIZE 970
SOCKET sock_server;

// 全局变量
AVFormatContext *fmt_ctx = NULL;
AVCodecContext *codec_ctx = NULL;
const AVCodec *codec = NULL;
int audio_stream_index = -1;
SDL_AudioDeviceID audio_device;
ifstream infile;
int wav_length = 0;
int result = 0;
char file_read[970];
// int 899 to {255,255, 255,26};
std::vector<unsigned char> decode_array(int number)
{
    int length = ceil(double(number) / 255);

    vector<unsigned char> deco_arr(4);

    for (int i = 0; i < 4; i++)
    {
        if (i < length - 1)
        {
            deco_arr[i] = 255;
        }
        else if (i == length - 1)
        {
            deco_arr[i] = number % 255;
        }
        else
        {
            deco_arr[i] = 0;
        }
        //   cout << int(deco_arr[i]) << " ";
    }
    // cout << endl;

    return deco_arr;
}
int send_index = 0;

// 音频回调函数
void audio_callback(void *userdata, Uint8 *stream, int len)
{

    static sockaddr_in addrClient;
    static int len_sock = sizeof(sockaddr_in);
    // 接收成功返回与client通讯的socket
    static SOCKET con = accept(sock_server, (SOCKADDR *)&addrClient, &len_sock);
    AVFrame *frame = (AVFrame *)userdata;
    static uint8_t audio_buf[(192000 * 3) / 2]; // 足够大的缓冲区
    static unsigned int audio_buf_size = 0;
    static unsigned int audio_buf_index = 0;

    while (len > 0)
    {
        if (audio_buf_index >= audio_buf_size)
        {
            // 读取新的音频帧
            audio_buf_index = 0;
            audio_buf_size = 970;
            vector<unsigned char> deco = decode_array(wav_length);
            char o1[1] = {'a'};
            for (int i = 0; i < 1; i++)
            {

                if (infile.read(file_read, 970))
                {

                    send_index += 1;
                }
                else
                {
                    printf("failed %d\n",infile.gcount());
                    infile.clear();

                 // infile.seekg(44,ios::beg);
                }
                send(con, file_read, sizeof(file_read), 0); // 给客户端发送一段信息
                int ret = recv(con, (char *)(audio_buf + i * 970), 970, 0);
                printf("ret===> %d %d %d\n", ret, result,audio_buf[1]);

                unsigned char d = audio_buf[1];
                for (int x = 0; x < d; x++)
                {
                    if (infile.read(file_read, 970))
                    {
    
                        send_index += 1;
                        send(con, file_read, sizeof(file_read), 0); // 给客户端发送一段信息
                        printf("compensation \n");
                    }
                }
            }
            result += 1;

            //    send(con, o1, sizeof(o1), 0); // 给客户端发送一段信息
            //     int ret = recv(con, (char *)audio_buf, 4096, 0);

            wav_length++;
        }

        int remaining = audio_buf_size - audio_buf_index;
        int to_copy = (remaining > len) ? len : remaining;
        memcpy(stream, audio_buf + audio_buf_index, to_copy);

        len -= to_copy;
        stream += to_copy;
        audio_buf_index += to_copy;
    }
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("usage: %s <audio file>\n", argv[0]);
        return -1;
    }

    const char *filename = argv[1];
    infile.open("queencard16000.wav", std::ios::binary | std::ios::ate);

    if (!infile.is_open())
    {
        printf("open file failed \n");
    }
    else
    {
        printf("open success \n");
    }

    infile.seekg(44);
    WSADATA wsaData;
    WORD wVersionRequested = MAKEWORD(2, 2);
    if (WSAStartup(wVersionRequested, &wsaData) != 0)
    {
        std::cout << "加载winsock.dll失败！" << std::endl;
        return 0;
    }
    // 创建套接字
    if ((sock_server = socket(AF_INET, SOCK_STREAM, 0)) == SOCKET_ERROR)
    {
        std::cout << "创建套接字失败！错误代码：" << WSAGetLastError() << std::endl;
        WSACleanup();
        return 0;
    }

    // 绑定端口和Ip
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.S_un.S_addr = inet_addr("192.168.1.7"); // 将点分十进制转化32位unsigned int

    if (SOCKET_ERROR == bind(sock_server, (SOCKADDR *)&addr, sizeof(sockaddr_in)))
    {
        std::cout << "地址绑定失败！错误代码：" << WSAGetLastError() << std::endl;
        closesocket(sock_server);
        WSACleanup();
        return 0;
    }
    // 将套接字设为监听状态
    listen(sock_server, 5);

    // 初始化 FFmpeg
    avformat_network_init();

    // 打开音频文件
    if (avformat_open_input(&fmt_ctx, filename, NULL, NULL) != 0)
    {
        printf("can not open : %s\n", filename);
        return -1;
    }

    // 查找流信息
    if (avformat_find_stream_info(fmt_ctx, NULL) < 0)
    {
        printf("无法获取流信息\n");
        return -1;
    }

    // 查找音频流
    for (int i = 0; i < fmt_ctx->nb_streams; i++)
    {
        if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
        {
            audio_stream_index = i;
            break;
        }
    }

    if (audio_stream_index == -1)
    {
        printf("未找到音频流\n");
        return -1;
    }

    // 获取解码器
    AVCodecParameters *codecpar = fmt_ctx->streams[audio_stream_index]->codecpar;
    codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec)
    {
        printf("未找到解码器\n");
        return -1;
    }

    // 创建解码器上下文
    codec_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codec_ctx, codecpar);
    if (avcodec_open2(codec_ctx, codec, NULL) < 0)
    {
        printf("无法打开解码器\n");
        return -1;
    }

    // 初始化 SDL
    if (SDL_Init(SDL_INIT_AUDIO) < 0)
    {
        printf("SDL初始化失败: %s\n", SDL_GetError());
        return -1;
    }

    printf("sample rate %d\n", codec_ctx->sample_rate);

    // 设置音频参数
    SDL_AudioSpec wanted_spec, obtained_spec;
    wanted_spec.freq = 16000;
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.channels = codec_ctx->channels;
    wanted_spec.silence = 0;
    wanted_spec.samples = SDL_AUDIO_BUFFER_SIZE;
    wanted_spec.callback = audio_callback;
    wanted_spec.userdata = av_frame_alloc();

    // 打开音频设备
    audio_device = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &obtained_spec, 0);
    if (audio_device == 0)
    {
        printf("无法打开音频设备: %s\n", SDL_GetError());
        return -1;
    }

    // 开始播放
    SDL_PauseAudioDevice(audio_device, 0);

    // 等待播放完成
    while (true)
    {
        SDL_Delay(100);
    }

    // 清理资源
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&fmt_ctx);
    SDL_CloseAudioDevice(audio_device);
    SDL_Quit();

    return 0;
}