#include "RosRpiInterface.h"

ROSRPIInterface::ROSRPIInterface(const std::string& ip, int port) : RPIInterface(ip, port), nh("~")
{
    initDefaultBuffers();
    adis_publisher = nh.advertise<sensor_msgs::Imu>("/adis", 10);
}
ROSRPIInterface::~ROSRPIInterface()
{
    adis_publisher.shutdown();
}

void ROSRPIInterface::initDefaultBuffers()
{
    // Initialize default ADIS buffer
    mADISBuffer = std::make_shared<RingBuffer>();
    // Initialize default ADIS TRIG buffer
    mADISTrigBuffer = std::make_shared<RingBuffer>();  
}

void ROSRPIInterface::BufferData()
{
    mStopped = false;
    int i = 0;
    
    while (!mStopped) {
        DataType type;
        size_t size = 32;
    
        mComms->readData(type, size);
        
        switch (type) {
            case DataType::ADI:
                // Push the entire buffer into the RingBuffer
                if (!mADISBuffer->pushBack(reinterpret_cast<uint8_t*>(mComms->buffer), size)) {
                    printf("ADIS Buffer Full!\n");
                }
                break;
            case DataType::ADIS_TRIG:
                // Push the entire buffer into the RingBuffer
                if (!mADISTrigBuffer->pushBack(reinterpret_cast<uint8_t*>(mComms->buffer), size)) {
                    printf("ADIS Trigger Buffer Full!\n");
                }
                break;
            case DataType::STATUS:
                mStopped = true;
                break;
            default:
                // Handle unknown data type
                break;
        }

        if(mADISTrigBuffer->getSize() >= (1 * size) && mADISBuffer->getSize() >= (2 * size)){
            mADISBufferReady.notify_one();
        }

    }
    mADISBufferReady.notify_one();
}

void ROSRPIInterface::PublishData()
{
    constexpr uint32_t size = CHUNK_SIZE;
    std::unique_ptr<uint8_t[]> adisData(new uint8_t[2 * size]);
    std::unique_ptr<uint8_t[]> adisTrigData(new uint8_t[size]);
    sensor_msgs::Imu imuMessage; // Create a single instance of sensor_msgs::Imu

    while (!mStopped) {
        // Wait for new data to become available in the ADIS buffer
        {
            std::unique_lock<std::mutex> lock(mADISBufferMutex);
            mADISBufferReady.wait(lock);
        }

        // Exit the loop if the interface is stopped
        if (mStopped) {
            break;
        }

        if (mADISBuffer->popFront(adisData.get(), 2 * size) && mADISTrigBuffer->popFront(adisTrigData.get(), size)) {
            // Process the data here
            RosMessage::adis(imuMessage, adisData.get(), adisTrigData.get());
            RosMessage::adis_print(imuMessage);
            adis_publisher.publish(imuMessage);

        } else {
            printf("Failed to pop data from the ADIS buffers.\n");
        }
    }
    printf("Done Publishing\n");
}
