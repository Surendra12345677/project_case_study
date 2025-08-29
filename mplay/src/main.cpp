#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include <opencv2/opencv.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
}
#include <alsa/asoundlib.h>

// Simple guard macros
#define CHECK_FF(call) do { int __r = (call); if (__r < 0) { char errbuf[256]; av_strerror(__r, errbuf, sizeof(errbuf)); \
  std::cerr << "FFmpeg error: " #call " -> " << errbuf << std::endl; std::exit(1);} } while(0)

static std::atomic<bool> g_running{true};

void handle_signal(int) { g_running = false; }

struct AudioSinkALSA {
    snd_pcm_t* handle = nullptr;
    unsigned int rate = 48000;
    int channels = 2;
    snd_pcm_format_t fmt = SND_PCM_FORMAT_S16_LE;

    bool open(const char* device = "default") {
        int rc = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0);
        if (rc < 0) {
            std::cerr << "ALSA open error: " << snd_strerror(rc) << std::endl;
            return false;
        }
        snd_pcm_hw_params_t* params;
        snd_pcm_hw_params_malloc(&params);
        snd_pcm_hw_params_any(handle, params);
        snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(handle, params, fmt);
        snd_pcm_hw_params_set_channels(handle, params, channels);
        unsigned int exact_rate = rate;
        snd_pcm_hw_params_set_rate_near(handle, params, &exact_rate, 0);
        if (exact_rate != rate) rate = exact_rate;
        // Set a reasonably large buffer to reduce XRUNs
        snd_pcm_uframes_t frames = 2048;
        snd_pcm_hw_params_set_period_size_near(handle, params, &frames, 0);
        int err = snd_pcm_hw_params(handle, params);
        snd_pcm_hw_params_free(params);
        if (err < 0) {
            std::cerr << "ALSA hw_params error: " << snd_strerror(err) << std::endl;
            return false;
        }
        snd_pcm_prepare(handle);
        return true;
    }

    // Write interleaved 16-bit samples. frames = samples_per_channel
    bool write(const int16_t* data, size_t frames) {
        while (frames > 0 && g_running.load()) {
            snd_pcm_sframes_t wrote = snd_pcm_writei(handle, data, frames);
            if (wrote == -EPIPE) {
                // underrun
                snd_pcm_prepare(handle);
                continue;
            } else if (wrote < 0) {
                std::cerr << "ALSA write error: " << snd_strerror(wrote) << std::endl;
                return false;
            }
            data += wrote * 2; // stereo
            frames -= wrote;
        }
        return true;
    }

    void close() {
        if (handle) {
            snd_pcm_drain(handle);
            snd_pcm_close(handle);
            handle = nullptr;
        }
    }
    ~AudioSinkALSA() { close(); }
};

static double to_seconds(int64_t pts, AVRational tb) {
    if (pts == AV_NOPTS_VALUE) return NAN;
    return pts * av_q2d(tb);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: mplay <media-file>" << std::endl;
        return 1;
    }
    std::string path = argv[1];

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    av_log_set_level(AV_LOG_ERROR);

    AVFormatContext* fmt = nullptr;
    CHECK_FF(avformat_open_input(&fmt, path.c_str(), nullptr, nullptr));
    CHECK_FF(avformat_find_stream_info(fmt, nullptr));

    int video_idx = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    int audio_idx = av_find_best_stream(fmt, AVMEDIA_TYPE_AUDIO, -1, -1, nullptr, 0);

    if (video_idx < 0 && audio_idx < 0) {
        std::cerr << "No audio/video streams found.\n";
        return 1;
    }

    // --- Video decoder ---
    AVCodecContext* vdec = nullptr;
    SwsContext* sws = nullptr;
    AVStream* vstream = nullptr;
    AVFrame* vframe = av_frame_alloc();

    if (video_idx >= 0) {
        vstream = fmt->streams[video_idx];
        const AVCodec* vcodec = avcodec_find_decoder(vstream->codecpar->codec_id);
        vdec = avcodec_alloc_context3(vcodec);
        CHECK_FF(avcodec_parameters_to_context(vdec, vstream->codecpar));
        CHECK_FF(avcodec_open2(vdec, vcodec, nullptr));
        sws = sws_getContext(vdec->width, vdec->height, vdec->pix_fmt,
                             vdec->width, vdec->height, AV_PIX_FMT_BGR24,
                             SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!sws) {
            std::cerr << "Failed to create SWS context.\n";
            return 1;
        }
        cv::namedWindow("mplay", cv::WINDOW_AUTOSIZE);
    }

    // --- Audio decoder ---
    AVCodecContext* adec = nullptr;
    SwrContext* swr = nullptr;
    AVStream* astream = nullptr;
    AVFrame* aframe = av_frame_alloc();
    AudioSinkALSA alsa;
    int64_t audio_ch_layout = AV_CH_LAYOUT_STEREO;
    int audio_rate = 48000;
    AVSampleFormat audio_fmt = AV_SAMPLE_FMT_S16;

    if (audio_idx >= 0) {
        astream = fmt->streams[audio_idx];
        const AVCodec* acodec = avcodec_find_decoder(astream->codecpar->codec_id);
        adec = avcodec_alloc_context3(acodec);
        CHECK_FF(avcodec_parameters_to_context(adec, astream->codecpar));
        CHECK_FF(avcodec_open2(adec, acodec, nullptr));

        // Setup resampler -> S16 stereo @ 48k
        swr = swr_alloc_set_opts(
            nullptr,
            audio_ch_layout,
            audio_fmt,
            audio_rate,
            adec->channel_layout ? adec->channel_layout : av_get_default_channel_layout(adec->channels),
            adec->sample_fmt,
            adec->sample_rate,
            0, nullptr
        );
        if (!swr || swr_init(swr) < 0) {
            std::cerr << "Failed to init SWR (audio resampler).\n";
            return 1;
        }
        if (!alsa.open("default")) {
            std::cerr << "Failed to open ALSA audio device.\n";
            return 1;
        }
        alsa.rate = audio_rate;
        alsa.channels = 2;
    }

    AVPacket* pkt = av_packet_alloc();

    // A/V sync (audio master)
    double audio_clock = 0.0; // seconds played
    double last_audio_update = av_gettime_relative() / 1e6;

    auto wall_time = [](){ return av_gettime_relative() / 1e6; };

    // BGR buffer for OpenCV
    std::vector<uint8_t> bgrbuf;
    AVFrame* rgb = av_frame_alloc();
    if (vdec) {
        int numBytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, vdec->width, vdec->height, 1);
        bgrbuf.resize(numBytes);
        av_image_fill_arrays(rgb->data, rgb->linesize, bgrbuf.data(), AV_PIX_FMT_BGR24, vdec->width, vdec->height, 1);
    }

    // Main demux/decode loop
    while (g_running.load() && av_read_frame(fmt, pkt) >= 0) {
        if (pkt->stream_index == video_idx && vdec) {
            CHECK_FF(avcodec_send_packet(vdec, pkt));
            AVFrame* f = vframe;
            int r;
            while ((r = avcodec_receive_frame(vdec, f)) >= 0) {
                // Convert to BGR for OpenCV
                sws_scale(sws, f->data, f->linesize, 0, vdec->height, rgb->data, rgb->linesize);
                double vpts = to_seconds(f->best_effort_timestamp, vstream->time_base);

                // Sync: wait until audio clock catches up (if we have audio)
                if (adec) {
                    // Update audio_clock based on wall time since last update (approximate when not writing)
                    double now = wall_time();
                    audio_clock += (now - last_audio_update);
                    last_audio_update = now;

                    // If video is ahead of audio by >30ms, wait a bit
                    if (!std::isnan(vpts) && vpts > audio_clock + 0.03) {
                        double delay = std::min(0.1, vpts - audio_clock);
                        std::this_thread::sleep_for(std::chrono::duration<double>(delay));
                        audio_clock += delay;
                    }
                } else {
                    // No audio -> simple frame pacing ~30 fps
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }

                // Show frame
                cv::Mat img(vdec->height, vdec->width, CV_8UC3, bgrbuf.data(), rgb->linesize[0]);
                cv::imshow("mplay", img);
                int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') { g_running = false; break; }
            }
            if (r != AVERROR(EAGAIN) && r != AVERROR_EOF && r < 0) {
                char errbuf[256]; av_strerror(r, errbuf, sizeof(errbuf));
                std::cerr << "Video receive_frame error: " << errbuf << std::endl;
            }
        } else if (pkt->stream_index == audio_idx && adec) {
            CHECK_FF(avcodec_send_packet(adec, pkt));
            int r;
            while ((r = avcodec_receive_frame(adec, aframe)) >= 0) {
                // Resample to S16 stereo @ 48k
                int out_chs = 2;
                int out_rate = 48000;
                int max_out = av_rescale_rnd(swr_get_delay(swr, adec->sample_rate) + aframe->nb_samples, out_rate, adec->sample_rate, AV_ROUND_UP);
                std::vector<int16_t> outbuf(max_out * out_chs);
                uint8_t* out_ptrs[1] = { reinterpret_cast<uint8_t*>(outbuf.data()) };
                int out_samples = swr_convert(swr, out_ptrs, max_out, (const uint8_t**)aframe->data, aframe->nb_samples);
                if (out_samples < 0) {
                    std::cerr << "swr_convert failed.\n";
                    continue;
                }
                // Write to ALSA
                if (!alsa.write(outbuf.data(), out_samples)) {
                    std::cerr << "ALSA write failed.\n";
                    g_running = false;
                    break;
                }
                // Update audio clock by the exact played duration
                audio_clock += static_cast<double>(out_samples) / out_rate;
                last_audio_update = wall_time();
            }
            if (r != AVERROR(EAGAIN) && r != AVERROR_EOF && r < 0) {
                char errbuf[256]; av_strerror(r, errbuf, sizeof(errbuf));
                std::cerr << "Audio receive_frame error: " << errbuf << std::endl;
            }
        }
        av_packet_unref(pkt);
        if (!g_running.load()) break;
    }

    // Flush decoders
    if (vdec) {
        avcodec_send_packet(vdec, nullptr);
        while (avcodec_receive_frame(vdec, vframe) >= 0) { /* drop */ }
    }
    if (adec) {
        avcodec_send_packet(adec, nullptr);
        while (avcodec_receive_frame(adec, aframe) >= 0) { /* drop */ }
    }

    // Cleanup
    if (vdec) { avcodec_free_context(&vdec); }
    if (adec) { avcodec_free_context(&adec); }
    if (sws) { sws_freeContext(sws); }
    if (swr) { swr_free(&swr); }
    if (fmt) { avformat_close_input(&fmt); }
    if (vframe) { av_frame_free(&vframe); }
    if (aframe) { av_frame_free(&aframe); }
    if (rgb) { av_frame_free(&rgb); }
    cv::destroyAllWindows();
    return 0;
}
