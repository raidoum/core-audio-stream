//
//  main.cpp
//  TestCoreAudio2
//
//  Created by Shohei Kinuta on 2023/03/20.
//
#include <CoreFoundation/CoreFoundation.h>
#include <CoreAudio/CoreAudio.h>
#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioComponent.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <thread>

#define USE_THREAD   (1)

// DUMP用のディレクトリ作成用　
#define MKDIR(name) mkdir(name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#define DUMMY_DATA_SIZE		(32)
FILE *fp_DumpAudio;
long pre_fp;
long cur_fp;
uint8_t dummy_data[DUMMY_DATA_SIZE] = {0};
uint8_t dump_data[4096 * 5];

// ダンプ用ファイルのopen/closeが重複呼び出しを防ぐため
bool isOpenDumpFile = false; 

// ダンプのディレクトリとファイル名の定義
#define DUMP_DIR_NAME  			("DUMP")
#define DUMP_FILE_NAME 			("audio")
#define DUMP_FILE_NAME_SIZE 	(50)

// CoreAudio系プロトタイプ宣言
void GetUSBAudioProperty(AudioObjectID*);
void GetStreamProperty(AudioObjectID);
void GetStreamProperty_v2(AudioObjectID);
void GetUIDProperty(AudioObjectID);
void GetStreamProperty_v3(AudioObjectID deviceID);
void CreateAudioUnit(AudioObjectID deviceID);
void pollLatency(AudioDeviceID deviceID);

// プロトタイプ宣言
bool open_file(char *, FILE **);
bool close_file(FILE **);
bool write_file(const uint8_t *, uint32_t, uint32_t, FILE **);
bool check_exist_file(char *);

// callback関数

// グローバル変数
AudioUnit audioUnit;
AudioBufferList* buf_list;
FILE *file;

bool isSnedStreamData = false;
uint32_t dump_audio_size = 0;
uint32_t req_dump_audio_size = 0;

// スレッド処理用
bool isStopStream = false;
uint32_t remain_cnt = 0;
uint8_t Wp = 0;
uint8_t Rp = 0;
uint32_t dbg_dump_cnt = 0;

using namespace std;

#if (USE_THREAD == 1)
void thread_FileWrite(void)
{
    while (1)
    {
        if (remain_cnt)
        {
            // sizeは暫定
            uint32_t size = 4096;

            write_file(dump_data + Rp * size, sizeof(uint8_t), size, &fp_DumpAudio);
            remain_cnt--;
            
            Rp++;
            if (Rp >= 5)
            {
                Rp = 0;
            }
        }

        if (isStopStream && remain_cnt == 0)
        {
            break;
        }
    }
}
#endif

bool open_file(char *base_name, FILE **fp)
{
    char file_name[DUMP_FILE_NAME_SIZE];

    for (int i = 0; i < 1000; i++)
    {
        sprintf(file_name, "%s_%03d.bin", base_name, i);
        if (!check_exist_file(file_name))
        {
            *fp = fopen(file_name, "ab");
            if (*fp != NULL)
            {
                // success create file
                return true;
            }
            else
            {
                // fail create file
                printf("create file: fail(%s)\n", file_name);
                return false;
            }
        }
    }
    //  exist base_name file over 1000 
    return false;
}

bool close_file(FILE **fp)
{
    int ret = fclose(*fp);

    if (ret != 0)
    {
        printf("close file: fail\n");
        return false;
    }

    return true;
}

bool write_file(const uint8_t* data, uint32_t unit, uint32_t size, FILE **fp)
{
	pre_fp = ftell(*fp);
    size_t written = fwrite(data, unit, size, *fp);

    if (written != size)
    {
        printf("write fail:(written=%zu, req=%u)\n", written, size);
        return false;
    }
	cur_fp = ftell(*fp);

	req_dump_audio_size += size;
	dump_audio_size += (uint32_t)written;
	
	long offset = cur_fp - pre_fp;
	if (offset != size)
	{
		printf("[Error] pre:%ld, cur:%ld, diff:%ld\n", pre_fp, cur_fp, offset);
	}

    return true;
}

bool check_exist_file(char *file_name)
{
    FILE *fp;
    fp = fopen(file_name, "r");
    if (fp != NULL)
    {
        // exist 
        return true;
    }
    else
    {
        // no exist
        fclose(fp);
        return false;
    }
}

// Audioデータに0以外の値が入っているかチェック
static bool check_stream_data(uint8_t* audio, uint32_t size)
{
	uint32_t *data = (uint32_t *)audio;
	uint32_t count = 0;
	
	for (uint32_t i = 0; i < size/4; i++)
	{
		uint32_t val = *data;
		data++;

		// check value
		if (val != 0)
		{
			uint32_t blank_size  = count * 32 / 8;
			printf("[Detect Stream Data] blank:%dbyte\n", blank_size);
			return true;
		}
		count++;
	}
	return false;
}
int main(int argc, const char * argv[]) {
    
    // DSCWEBCAMのAudioデバイスIDを取得
    AudioObjectID dscwebcam_id = kAudioObjectUnknown;
#if 1
    GetUSBAudioProperty(&dscwebcam_id);
    if (dscwebcam_id == kAudioObjectUnknown)
    {
        cout << "DSCWEBCAN not Found in AudioDevice"<< endl;
        return 0;
    }
#endif
#if (USE_THREAD == 1)
    // ファイルをダンプするためのスレッドを起動
    thread th(thread_FileWrite);
#endif

    GetUIDProperty(dscwebcam_id);
    //GetStreamProperty_v3(dscwebcam_id);
    GetStreamProperty_v2(dscwebcam_id);
    CreateAudioUnit(dscwebcam_id);
    
    // キー入力待ち
    getchar();

    OSStatus stat = AudioOutputUnitStop(audioUnit);
    if (stat != noErr)
    {
        cout << "error(start): " << stat << endl;
    }
    
#if (USE_THREAD == 1)
    isStopStream = true;
    
    // スレッド終了待ち
    th.join();
#endif
    
    // ファイルクローズ
	if (isOpenDumpFile)
	{
    	bool ret = close_file(&fp_DumpAudio);
    	if (!ret)
    	{
			// close fail
    	    printf("close file: fail\n");
    	}
		else
		{
			// close success
			isOpenDumpFile = false;
		}
		printf("dump size (%d / %d) byte\n", dump_audio_size, req_dump_audio_size);
        printf("notify size (%d) byte\n", dbg_dump_cnt * 4096); //4096は暫定
	}

    // DSCWEBCAMのストリーム情報を取得
    
    return 1;
}

void pollLatency(AudioDeviceID deviceID)
{
    UInt32 latencyFrames = 0;
    UInt32 propertySize = sizeof(latencyFrames);
    AudioObjectPropertyAddress propertyAddresss = {
        kAudioDevicePropertyLatency,
        kAudioObjectPropertyScopeInput,
        kAudioObjectPropertyElementMain
    };
    
    OSStatus stat = AudioObjectGetPropertyData(deviceID, &propertyAddresss, 0, nullptr, &propertySize, &latencyFrames);
    if (stat != noErr)
    {
        cout << "Error: get latency" << endl;
        return;
    }
    cout << "latency: " << latencyFrames << endl;
}

OSStatus input_callback(void *data,
                   AudioUnitRenderActionFlags *action_flags,
                   const AudioTimeStamp *ts_data, UInt32 bus_num,
                   UInt32 frames, AudioBufferList *ignored_buffers)
{
    OSStatus stat = AudioUnitRender(audioUnit, action_flags, ts_data, bus_num, frames,
                       buf_list);
    if (stat != noErr)
    {
        cout << "Error(Rneder): " << stat << endl;
        return stat;
    }
    
	long sec;
	long nsec;
	double d_sec = 0;

	struct timespec cur_time;
	struct timespec static pre_time;
	static bool isFirst = true;
    static uint32_t interval_over_cnt = 0;

	clock_gettime(CLOCK_REALTIME, &cur_time);
	if (isFirst)
	{
		isFirst = false;
	}
	else
	{
		sec  = cur_time.tv_sec - pre_time.tv_sec;
		nsec = cur_time.tv_nsec - pre_time.tv_nsec;
		d_sec = (double)sec + (double)nsec / (1000 * 1000 *1000);
		d_sec *= 1000;
        
	}
	pre_time.tv_sec = cur_time.tv_sec;
	pre_time.tv_nsec = cur_time.tv_nsec;
    
    uint8_t *audio = (uint8_t *)buf_list->mBuffers[0].mData;
    uint32_t size = buf_list->mBuffers->mDataByteSize;
    
	if (!isSnedStreamData)
	{
		isSnedStreamData = check_stream_data(audio, size);
	}
	if (isSnedStreamData)
	{
		if (d_sec >= 11)
		{
            interval_over_cnt++;
			printf("[interval] over interval:%f, cnt:%d\n", d_sec, interval_over_cnt);
			//write_file(dummy_data, sizeof(uint8_t), DUMMY_DATA_SIZE, &fp_DumpAudio);
		}
			
		// 処理時間の計算
		double d_cpu_sec;
		static double d_cpu_sec_max = 0;

		clock_t start_cpu_time, end_cpu_time;
		
		start_cpu_time = clock();
#if (USE_THREAD == 1)
        if (remain_cnt > 5)
        {
            cout << "buffer overflow" << endl;
        }
        memcpy(dump_data + Wp * size, audio, size);
        remain_cnt++;

        Wp++;
        if (Wp >= 5)
        {
            Wp = 0;
        }
        dbg_dump_cnt++;
#else
		write_file(audio, sizeof(uint8_t), size, &fp_DumpAudio);
#endif
		end_cpu_time = clock();
		
		d_cpu_sec = (double)(end_cpu_time - start_cpu_time) / CLOCKS_PER_SEC * 1000;

		if (d_cpu_sec_max < d_cpu_sec)
		{
			d_cpu_sec_max = d_cpu_sec;
		}
		//printf("[process time] time=%fsec, clock=%fms\n", d_sec, d_cpu_sec, d_cpu_sec_max);
		//printf("[process time] int=%fms, clock=%fms, clock_max=%fms\n", d_sec, d_cpu_sec, d_cpu_sec_max);
	}
    //UInt32 *data = (uint32_t *)buf_list->mBuffers[0].mData;
    // ファイル書き込み
    //fwrite(buf_list->mBuffers[0].mData, 1, buf_list->mBuffers[0].mDataByteSize, file);
    
    
    return noErr;
}

OSStatus on_changed_input_audio_device(
                                       AudioObjectID deviceID,
                                       UInt32 InNumberAddress,
                                       const AudioObjectPropertyAddress* inAddress,
                                       void* inClientData)
{
    cout << "Notify Change Property " << endl;
    
    UInt32 size = sizeof(UInt32);
    UInt32 lantecy;
    AudioObjectGetPropertyData(deviceID, inAddress, 0, nullptr, &size, &lantecy);
    
    cout << lantecy << endl;
    
    return noErr;
}

// UsBAudioデバイスの中から[DSCWEBCAM]のデバイスIDを取得する
void GetUSBAudioProperty(AudioObjectID *target_deviceID)
{
    UInt32 property_size;
    AudioObjectPropertyAddress property_address = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain,
    };
    
    // すべてのプロパティサイズを取得
    OSStatus status = AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &property_address, 0, nullptr, &property_size);
    UInt32 device_cnt = property_size / sizeof(AudioObjectID);
    vector<AudioObjectID> device_list(device_cnt);
    
    // すべてのプロパティ情報を取得
    status = AudioObjectGetPropertyData(kAudioObjectSystemObject, &property_address, 0, nullptr, &property_size, &device_list[0]);
    
    // すべてのプロパティの情報からUSB接続されているデバイスIDを見つける
    AudioObjectPropertyAddress transportTypeAddress = {
        kAudioDevicePropertyTransportType,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    
    vector<AudioObjectID> usb_audio_device_list(0);
    for (UInt32 i = 0; i < device_cnt; i++){
        UInt32 transport_type = kAudioDeviceTransportTypeUnknown ;
        UInt32 size = sizeof(transport_type);
        status = AudioObjectGetPropertyData(device_list[i], &transportTypeAddress, 0, nullptr, &size, &transport_type);
        
        if (status == noErr)
        {
            if (transport_type == kAudioDeviceTransportTypeUSB)
            {
                // USBAudioデバイスIDを取得
                usb_audio_device_list.push_back(device_list[i]);
            }
        }
    }
    
    // USBAudioデバイス名を取得する
     AudioObjectPropertyAddress deviceNameAddress = {
        kAudioDevicePropertyDeviceName,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    
    char deviceName[256];
    // 他のデバイスを見たいときは名前を変更
    char target_deviceName[] = "DSC";
    UInt32 dname_property_size = sizeof(deviceName);
    
    for (auto usb_audio_device : usb_audio_device_list)
    {
        status = AudioObjectGetPropertyData(usb_audio_device, &deviceNameAddress, 0, nullptr, &dname_property_size, &deviceName);
        
        //if (strcmp(deviceName, target_device_name, 3) ==  0)
        if (strncmp(deviceName, target_deviceName, 3) == 0)
        {
            *target_deviceID = usb_audio_device;
            cout << deviceName << ": " << usb_audio_device << endl;
        }
       // cout << usb_audio_device << endl;
    }
    
    //return status;
}

void GetStreamProperty(AudioObjectID device_id)
{
    AudioObjectPropertyAddress address = {
        kAudioDevicePropertyStreams,
        //kAudioStreamPropertyVirtualFormat,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    
    UInt32 size = 0;
    // DSCWEBCAMののストリーム一覧のサイズを取得
    OSStatus status = AudioObjectGetPropertyDataSize(device_id, &address, 0, nullptr, &size);
    if (status != noErr)
    {
        cout << "AudioObjectGetPropertyDataSize_Error code: " << status << " " << "device_id: " << device_id << endl;
        return;
    }
    vector<AudioObjectID> streams(size / sizeof(AudioObjectID));
    
    // DSCWEBCAMのストリーム一覧を取得
    status = AudioObjectGetPropertyData(device_id, &address, 0, nullptr, &size, &streams[0]);
     if (status != noErr)
    {
        cout << "AudioObjectGetPropertyData_Error code: " << status << " " << "device_id: " << device_id << endl;
        return;
    }
    
    AudioObjectID input_stream = kAudioObjectUnknown;
    address.mSelector = kAudioStreamPropertyDirection;
    
    //ストリームのリストから入力ストリームを取得
    for (auto stream: streams)
    {
        UInt32 direction = 0;
        size = sizeof(direction);
        AudioObjectGetPropertyData(stream, &address, 0, nullptr, &size, &direction);
         if (direction == 1)
        {
            input_stream = stream;
            break;
        }
    }
    if (input_stream == kAudioObjectUnknown)
    {
        cout << "No OutputStream" << endl;
        return;
    }
    else
    {
        cout << input_stream << endl;
    }
    
    address.mSelector = kAudioStreamPropertyVirtualFormat;
    AudioStreamBasicDescription asbd;
    size = sizeof(asbd);

    // 入力ストリームのAudioStreamBasicDescriptionの取得
    status = AudioObjectGetPropertyData(input_stream, &address, 0, nullptr, &size, &asbd);
    if (status != noErr)
    {
        cout << "Error code: " << status << " " << "device_id: " << device_id << endl;
    }
    
}
void GetStreamProperty_v2(AudioObjectID device_id)
{
    AudioObjectPropertyAddress address = {
        kAudioDevicePropertyStreams,
        kAudioObjectPropertyScopeInput,
        kAudioObjectPropertyElementMain
    };
    
    UInt32 size = 0;
    // DSCWEBCAMののストリーム一覧のサイズを取得
    OSStatus status = AudioObjectGetPropertyDataSize(device_id, &address, 0, nullptr, &size);
    if (status != noErr)
    {
        cout << "AudioObjectGetPropertyDataSize_Error code: " << status << " " << "device_id: " << device_id << endl;
        return;
    }
    vector<AudioObjectID> streams(size / sizeof(AudioObjectID));
    
    // DSCWEBCAMのストリーム一覧を取得
    status = AudioObjectGetPropertyData(device_id, &address, 0, nullptr, &size, &streams[0]);
    if (status != noErr)
    {
        cout << "AudioObjectGetPropertyData_Error code: " << status << " " << "device_id: " << device_id << endl;
        return;
    }
    
    address.mSelector = kAudioDevicePropertyStreamFormat;
    //address.mSelector = kAudioStreamPropertyPhysicalFormat;
    //address.mSelector = kAudioStreamPropertyVirtualFormat;
    //address.mScope = kAudioObjectPropertyScopeInput;
    //address.mElement = kAudioObjectPropertyElementMain;
    
#if 0
     // Streamプロパティ情報を変更できるか確認
    Boolean result = false;
    AudioObjectIsPropertySettable(device_id, &address, &result);
    if (result)
    {
        // 設定するプロパティ情報
        AudioStreamBasicDescription audioFormat;
        size = sizeof(audioFormat);
        memset(&audioFormat, 0, size);
        audioFormat.mSampleRate = 48000;
        audioFormat.mFormatID = kAudioFormatLinearPCM;
        audioFormat.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
        audioFormat.mBitsPerChannel = 16;
        audioFormat.mChannelsPerFrame = 2;
        audioFormat.mBytesPerFrame = audioFormat.mBitsPerChannel / 8 * audioFormat.mChannelsPerFrame;
        audioFormat.mFramesPerPacket = 1;
        audioFormat.mBytesPerPacket = audioFormat.mBytesPerFrame * audioFormat.mFramesPerPacket;
        
        status = AudioObjectSetPropertyData(device_id, &address, 0, nullptr, size, &audioFormat);
        if (status != noErr)
        {
            cout << "Error" << endl;
        }
    }
    else
    {
        cout << "Cant't change Property" << endl;
    }
#endif
    
    //cout << streams[0] << endl;
    // Streamプロパティ取得
    //address.mSelector = kAudioStreamPropertyVirtualFormat;
    //address.mElement = streams[0];
    //status = AudioObjectGetPropertyDataSize(streams[0], &address, 0, nullptr, &size);
    status = AudioObjectGetPropertyDataSize(streams[0], &address, 0, nullptr, &size);
    if (status != noErr)
    {
        cout << "Error "<< endl;
        return;
    }
    AudioStreamBasicDescription streamFormat;
    status = AudioObjectGetPropertyData(streams[0], &address, 0, nullptr, &size, &streamFormat);
    if (status != noErr)
    {
        cout << "Error" << endl;
    }
}
void GetStreamProperty_v3(AudioObjectID deviceID)
{
    // USBオーディオデバイスの持つストリームIDを取得する
    UInt32 propertySize = 0;
    OSStatus err = noErr;

    // kAudioDevicePropertyStreamsプロパティを使用して、USBオーディオデバイスが持つストリームIDを取得する
    err = AudioDeviceGetPropertyInfo(deviceID, 0, true, kAudioDevicePropertyStreams, &propertySize, nullptr);
    
    int numStreams = propertySize / sizeof(AudioStreamID);
    AudioStreamID *streamIDs = (AudioStreamID *)malloc(propertySize);

    // ストリームIDの取得
    err = AudioDeviceGetProperty(deviceID, 0, true, kAudioDevicePropertyStreams, &propertySize, streamIDs);
    
    AudioStreamBasicDescription streamFormat;
    propertySize = sizeof(streamFormat);
    AudioStreamGetProperty(streamIDs[0], 0, kAudioDevicePropertyStreamFormat, &propertySize, &streamFormat);
    cout << numStreams << endl;
    
    free(streamIDs);

}

void GetUIDProperty(AudioObjectID device_id)
{
    AudioObjectPropertyAddress address = {
        kAudioDevicePropertyDeviceUID,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    CFStringRef deviceUID = nullptr;
    UInt32 size = sizeof(deviceUID);
    AudioObjectGetPropertyData(device_id, &address, 0, nullptr, &size, &deviceUID);
    if (deviceUID != NULL) {
          CFIndex length = CFStringGetLength(deviceUID);
          CFIndex maxSize = CFStringGetMaximumSizeForEncoding(length, kCFStringEncodingUTF8);
          char buffer[maxSize];
          if (CFStringGetCString(deviceUID, buffer, maxSize, kCFStringEncodingUTF8)) {
              std::cout << buffer << std::endl;
          }
          CFRelease(deviceUID);
      }
}
void CreateAudioUnit(AudioObjectID deviceID)
{
    OSStatus stat;
    AudioComponentDescription desc;
    desc.componentType = kAudioUnitType_Output;
    desc.componentSubType = kAudioUnitSubType_HALOutput;
    desc.componentManufacturer = 0;
    desc.componentFlags = 0;
    desc.componentFlagsMask = 0;
    
    AudioComponent component = AudioComponentFindNext(NULL, &desc);
    if (!component)
    {
        cout << "Cna't find component" << endl;
    }
    
    // AudioUnit
    AudioComponentInstanceNew(component, &audioUnit);
    
    //Input
    UInt32 enableInput = 1;
    AudioUnitElement inputBus = 1;
    stat = AudioUnitSetProperty(audioUnit, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, inputBus, &enableInput, sizeof(enableInput));
    if (stat != noErr)
    {
        cout << "error1: " << stat << endl;
    }
    
    //Output
    UInt32 enableOutput = 0;
    AudioUnitElement outputBus = 0;
    stat = AudioUnitSetProperty(audioUnit, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, outputBus, &enableOutput, sizeof(enableOutput));
    if (stat != noErr)
    {
        cout << "error2: " << stat << endl;
    }
    
    //AudioUnit - deviceID
    stat = AudioUnitSetProperty(audioUnit, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &deviceID, sizeof(deviceID));
    if (stat != noErr)
    {
        cout << "error3: " << stat << endl;
    }
    
    // Set Format
    AudioStreamBasicDescription fdesc;
    UInt32 size = sizeof(fdesc);
#if 1
    AudioStreamBasicDescription audioFormat;
    memset(&audioFormat, 0, size);
    audioFormat.mSampleRate = 48000;
    audioFormat.mFormatID = kAudioFormatLinearPCM;
    //audioFormat.mFormatFlags = kAudioFormatFlagIsPacked;
    audioFormat.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    audioFormat.mBitsPerChannel = 16;
    audioFormat.mChannelsPerFrame = 2;
    audioFormat.mBytesPerFrame = audioFormat.mBitsPerChannel / 8 * audioFormat.mChannelsPerFrame;
    audioFormat.mFramesPerPacket = 1;
    audioFormat.mBytesPerPacket = audioFormat.mBytesPerFrame * audioFormat.mFramesPerPacket;
#endif
    
    stat = AudioUnitGetProperty(audioUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, inputBus, &fdesc, &size);
    //stat = AudioUnitGetProperty(audioUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, inputBus, &fdesc, &size);
    if (stat != noErr)
    {
        cout << "error(get Format): " << stat << endl;
    }
#if 0
    stat = AudioUnitSetProperty(audioUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, inputBus, &audioFormat, size);
    if (stat != noErr)
    {
        cout << "error(set Format1): " << stat << endl;
    }
#endif
    
    stat = AudioUnitSetProperty(audioUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, inputBus, &fdesc, size);
    if (stat != noErr)
    {
        cout << "error(set Format2): " << stat << endl;
    }
    
    // set buffer
    UInt32 frames = 0;
    UInt32 buf_size = 0;
    
    AudioObjectPropertyAddress addr = {
        kAudioDevicePropertyStreamConfiguration,
        kAudioDevicePropertyScopeInput,
        kAudioObjectPropertyElementMain
    };
    stat = AudioObjectGetPropertyDataSize(deviceID, &addr, 0, nullptr, &buf_size);
    
    size = sizeof(frames);
    
    stat = AudioUnitGetProperty(audioUnit, kAudioDevicePropertyBufferSize, kAudioUnitScope_Global, 0, &frames, &size);
    
    buf_list = (AudioBufferList *)malloc(buf_size);
    
    stat = AudioObjectGetPropertyData(deviceID, &addr, 0, nullptr, &buf_size, &buf_list[0]);
    
    for (UInt32 i = 0; i < buf_list->mNumberBuffers; i++)
    {
        size = buf_list->mBuffers[i].mDataByteSize;
        buf_list->mBuffers[i].mData = malloc(size);
    }
    
    AURenderCallbackStruct callback_info = {.inputProc = input_callback,
        .inputProcRefCon = audioUnit};
    
    // コールバック関数の登録
    stat = AudioUnitSetProperty(audioUnit, kAudioOutputUnitProperty_SetInputCallback, kAudioUnitScope_Global, 0, &callback_info, sizeof(callback_info));
    if (stat != noErr)
    {
        cout << "error(set callback): " << stat << endl;
    }
    
    //初期化
    stat = AudioUnitInitialize(audioUnit);
    if (stat != noErr)
    {
        cout << "error(Initialized): " << stat << endl;
    }
    
    MKDIR(DUMP_DIR_NAME);
    char base_name[DUMP_FILE_NAME_SIZE]; 
    sprintf(base_name, "%s/%s", DUMP_DIR_NAME, DUMP_FILE_NAME);
    
    // ダンプ用ファイル作成
	if (!isOpenDumpFile)
	{
    	bool ret = open_file(base_name, &fp_DumpAudio);
    	if (!ret)
    	{
			// open fail
    	    printf("open file: fail\n");
    	}
		else
		{
			// open success
			isOpenDumpFile = true;
		}
	}
    
    //ストリーム開始
    stat = AudioOutputUnitStart(audioUnit);
    if (stat != noErr)
    {
        cout << "error(start): " << stat << endl;
    }
    
    
#if 0
    // 遅延リセットされる遅延量のしきい値を確認するために遅延量プロパティをポーリングする
    AudioObjectPropertyAddress latePropertyAddr = {
        kAudioDevicePropertyLatency,
        kAudioObjectPropertyScopeInput,
        kAudioObjectPropertyElementMain
    };
    size = sizeof(UInt32);
    UInt32 initialLatency;
    // 最初の遅延量を取得
    stat = AudioObjectGetPropertyData(deviceID, &latePropertyAddr, 0, nullptr, &size, &initialLatency);
    if (stat != noErr)
    {
        cout << "Erro: get initial latency" << endl;
        return;
    }
    cout << "InitLantecy: " << initialLatency << endl;
    
    // コールバック関数の登録
    stat = AudioObjectAddPropertyListener(deviceID, &latePropertyAddr, on_changed_input_audio_device, nullptr);
    if (stat != noErr)
    {
        cout << "Error: Register Callback" << endl;
    }
#endif
}
