#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>

#include "RosMessage.h"
#include "OptitrackInterface.h"

class RosOptiInterface : public OptitrackInterface {

protected:

	static std::shared_ptr<geometry_msgs::PoseStamped> static_pose;

    ros::NodeHandle nh; // ROS node handle
    static ros::Publisher opti_publisher; // Publisher object
public:

    RosOptiInterface();
    ~RosOptiInterface();

    // Override InitOptiTrack to use DataPublisher instead of DataHandler
    void setCallbackFunction() override;

    // New static function to be used instead of DataHandler
    static void NATNET_CALLCONV DataPublisher(sFrameOfMocapData* data, void* pUserData)
	{
		
		clientLatency = 0;
		totalLatency = 0;
		transitLatency = 0;
		hour = 0;
		minute = 0;
		second = 0;
		frame = 0;
		subframe = 0;

		NatNetClient* pClient = (NatNetClient*)pUserData;

		// Software latency here is defined as the span of time between:
		//   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
		// and
		//   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
		//
		// This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
		// because it additionally includes the time spent preparing to stream the data via NatNet.
		//const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
		//const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

		// Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
		// The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
		const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;
		//transitLatency_Millisec = transitLatencyMillisec;
		 /*if (g_outputFile)
		 {
			 _WriteFrame(g_outputFile, data);
		 }
*/
		int i = 0;

		// printf("FrameID : %d\n", data->iFrame);
		 //printf("Timestamp : %3.2lf\n", data->fTimestamp);
		 //printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

		 // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
		 // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
		const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

		if (bSystemLatencyAvailable)
		{
			// System latency here is defined as the span of time between:
			//   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
			// and
			//   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
			//const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
		    //const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

			// Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
			// This is the all-inclusive measurement (photons to client processing).
			const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;

			//clientLatency_Millisec = clientLatencyMillisec;
			// You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
			//const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

			//printf("System latency : %.2lf milliseconds\n", systemLatencyMillisec);
		    //printf("Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec);
		    transitLatency = transitLatencyMillisec;
		    clientLatency = clientLatencyMillisec;

		    totalLatency = clientLatencyMillisec + transitLatencyMillisec;

		}
		else
		{
			// printf("Transit latency : %.2lf milliseconds\n", transitLatencyMillisec);
		}

		// FrameOfMocapData params
		bool bIsRecording = ((data->params & 0x01) != 0);
		bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
		/*if (bIsRecording)
			printf("RECORDING\n");
		if (bTrackedModelsChanged)
			printf("Models Changed.\n");*/


		// timecode - for systems with an eSync and SMPTE timecode generator - decode to values
		//int hour, minute, second, frame, subframe;
		NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
		// decode to friendly string
		char szTimecode[128] = "";
		NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);

		// printf("Timecode : %s\n", szTimecode);

		 // Rigid Bodies
		 //printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
		for (i = 0; i < data->nRigidBodies; i++)
		{
			if (data->RigidBodies->ID == _optiID && data->RigidBodies[i].params & 0x01)
			{
				// params
				// 0x01 : bool, rigid body was successfully tracked in this frame
				_mtx.lock();
				static_pose->pose.position.x = data->RigidBodies[i].x;
				static_pose->pose.position.y = data->RigidBodies[i].y;
				static_pose->pose.position.z = data->RigidBodies[i].z;
				static_pose->pose.orientation.x = data->RigidBodies[i].qx;
				static_pose->pose.orientation.y = data->RigidBodies[i].qy;
				static_pose->pose.orientation.z = data->RigidBodies[i].qz;
				static_pose->pose.orientation.w = data->RigidBodies[i].qw;

			    _mtx.unlock();

				long long ns_count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()
                ).count();

				static_pose->header.stamp = ros::Time().fromNSec(ns_count);
				opti_publisher.publish(*static_pose);
			}
		}
		
		if ((isinf(transitLatency))||(transitLatency > 50.0))
			printf("opt: transitLatency %lf\n", transitLatency);
			
		if ((isinf(clientLatency))||(clientLatency > 50.0))
			printf("opt: clientLatency %lf\n", clientLatency);
	}
};
