1本项目使用stm32f407zgt6的lwip模块使用tcp发送和接收音频,i2s2用于生成录音，i2s3用于播放接收到的音频，类似于微信语音聊天。
2服务端使用SDL2模块和ffmpeg模块播放和接收音频，服务端循环发送queencard16000.wav并播放来自stm32的录音，stm32接收来自服务端的queercard16000音乐并播放
3 i2s2和i2s3都用到了dma双缓存机制
4 编译g++  play_audio_wav_stm32.cpp -o play -I  -I D:\youffmpeg\include -L D:\youffmpeg\lib -lavcodec -lavformat -lavutil -lswscale -lswresample  -I D:\yousdl2\include -L D:\yousdl2\bin -lSDL2  -lwsock32 -D__STDC_FORMAT_MACROS -std=c++11
5 运行playaudio.exe queencard.wav
