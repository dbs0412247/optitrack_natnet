//=============================================================================
// Copyright © 2014 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================

// System includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

// Local includes 
#include "optitrack_natnet/socket.h" 

// NatNetSDK includes
#include "NatNetSDK/NatNetTypes.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>

// Constants
#define MULTICAST_IP    "239.255.42.99"
#define DATA_PORT       1511
#define CMD_PORT        1510
#define SHUTTLECOCK_ID  1
#define STEREO_CAM_ID   2

ros::Time first_ros_time;
ros::Time first_timestamp;
bool first_observation = true;

void broadcastRecordTfData(tf::TransformBroadcaster &br, sFrameOfMocapData &data, rosbag::Bag &bag) {
  // calculate correct timestamp of msg
  if (first_observation) {
    first_observation = false;
    first_ros_time = ros::Time::now();
    first_timestamp = ros::Time(data.fTimestamp);
  }
  ros::Duration timestamp_delta = ros::Time(data.fTimestamp) - first_timestamp;
  ros::Time time_of_data = first_ros_time + timestamp_delta; 
  bool bIsRecording = data.params & 0x01;
  std::vector< geometry_msgs::TransformStamped > vec_msg;
  tf::tfMessage tfmsg;
  // broadcast each Rigid Body
  for (int i = 0; i < data.nRigidBodies; i++) {
    if (data.RigidBodies[i].params & 0x01) { // bTrackingValid
      //printf("...Seen\n");
      char name[20];
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(
        data.RigidBodies[i].x, 
        data.RigidBodies[i].y,
        data.RigidBodies[i].z) );
      tf::Quaternion q(
        data.RigidBodies[i].qx, 
        data.RigidBodies[i].qy,
        data.RigidBodies[i].qz,
        data.RigidBodies[i].qw);
      transform.setRotation(q);
      sprintf(name, "Rigid_Body_ID_%d", data.RigidBodies[i].ID); 
      tf::StampedTransform stamped_tf = tf::StampedTransform(transform, time_of_data, "world", name); 
      if (data.RigidBodies[i].ID == 1) 
        stamped_tf = tf::StampedTransform(transform, time_of_data, "world", "Shuttlecock");
      else if (data.RigidBodies[i].ID == 2)  
        stamped_tf = tf::StampedTransform(transform, time_of_data, "world", "Stereo Camera"); 
      else 
        stamped_tf = tf::StampedTransform(transform, time_of_data, "world", name); 
      br.sendTransform(stamped_tf);
      // start recording to rosbag if mocap is also recording
      if (bIsRecording) {
        // converting a tf::Transform into a geometry_msgs::TransformStamped,
        // then into a tf::tfMessage for writing into rosbag        
        geometry_msgs::TransformStamped msg;
        tf::transformStampedTFToMsg(stamped_tf, msg);
        bag.write("/tf", time_of_data, msg);
      }    
    }  // end if bTrackingValid 
  } // end for
}

bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe)
{
  bool bValid = true;

  *hour = (inTimecode>>24)&255;
  *minute = (inTimecode>>16)&255;
  *second = (inTimecode>>8)&255;
  *frame = inTimecode&255;
  *subframe = inTimecodeSubframe;

  return bValid;
}

bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize)
{
  bool bValid;
  int hour, minute, second, frame, subframe;
  bValid = DecodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

  snprintf(Buffer,BufferSize,"%2d:%2d:%2d:%2d.%d",hour, minute, second, frame, subframe);
  for(unsigned int i=0; i<strlen(Buffer); i++)
          if(Buffer[i]==' ')
                  Buffer[i]='0';

  return bValid;
}

void destroyRigidBody(sRigidBodyData &data) {
  free(data.Markers);
  free(data.MarkerIDs);
  free(data.MarkerSizes);
}

void destroySkeleton(sSkeletonData &data) {
  for (int i = 0; i < data.nRigidBodies; i++) 
    destroyRigidBody(data.RigidBodyData[i]);
  if (data.nRigidBodies) 
    free(data.RigidBodyData);
}

void destroyDataPacket(sFrameOfMocapData &data) {
  for (int i = 0; i < data.nMarkerSets; i++) 
    free(data.MocapData[i].Markers); 
  if (data.nOtherMarkers) 
   free(data.OtherMarkers);
  for (int i = 0; i < data.nRigidBodies; i++) 
    destroyRigidBody(data.RigidBodies[i]);
  for (int i = 0; i < data.nSkeletons; i++) 
    destroySkeleton(data.Skeletons[i]);
}

bool parseDataPacket(const char* pData, sFrameOfMocapData &data, sDataDescriptions &desc) {
  

  // TODO: figure out how to get NatNetVersion
  int major = 2; 
  int minor = 7;

  char *ptr = (char *)pData;
 
  // message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2); ptr += 2;

  // size
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2); ptr += 2;
  
  if(MessageID == NAT_FRAMEOFDATA) {      // FRAME OF MOCAP DATA packet (7)
    // frame number
    memcpy(&(data.iFrame), ptr, 4); ptr += 4;
    //printf("iFrame = %d\n", data.iFrame);
    	
    // number of data sets (markersets, rigidbodies, etc)
    memcpy(&(data.nMarkerSets), ptr, 4); ptr += 4;
    //printf("nMarkerSets = %d\n", data.nMarkerSets);
    for (int i=0; i < data.nMarkerSets; i++) {    
      // Markerset name
      strcpy(data.MocapData[i].szName, ptr);
      int nDataBytes = (int) strlen(data.MocapData[i].szName) + 1;
      ptr += nDataBytes;

      // marker data
      int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
      data.MocapData[i].nMarkers = nMarkers;
      int nBytes = nMarkers*sizeof(float[3]);
      data.MocapData[i].Markers = (float(*)[3])malloc(nBytes);
      memcpy(data.MocapData[i].Markers, ptr, nBytes);
      ptr += nBytes; 
    }

    // unidentified markers
    int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
    data.nOtherMarkers = nOtherMarkers;
    //printf("nOtherMarkers = %d\n", data.nOtherMarkers);
    int nOtherMarkerBytes = nOtherMarkers*sizeof(float[3]);
    data.OtherMarkers = (float(*)[3])malloc(nOtherMarkerBytes);
    memcpy(data.OtherMarkers, ptr, nOtherMarkerBytes);
    ptr += nOtherMarkerBytes;
            
    // rigid bodies
    memcpy(&(data.nRigidBodies), ptr, 4); ptr += 4;
    //printf("nRigidBodies = %d\n", data.nRigidBodies);
    for (int j=0; j < data.nRigidBodies; j++) {
      // rigid body pos/ori
      int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
      float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
      float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
      float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
      float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
      float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
      float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
      float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
      data.RigidBodies[j].ID = ID;
      data.RigidBodies[j].x = x;
      data.RigidBodies[j].y = y;
      data.RigidBodies[j].z = z;
      data.RigidBodies[j].qx = qx;
      data.RigidBodies[j].qy = qy;
      data.RigidBodies[j].qz = qz;
      data.RigidBodies[j].qw = qw;

      // associated marker positions
      int nRigidMarkers = 0; memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
      data.RigidBodies[j].nMarkers = nRigidMarkers;
      
      int nBytes = nRigidMarkers*sizeof(float[3]);
      data.RigidBodies[j].Markers = (float(*)[3])malloc(nBytes);
      memcpy(data.RigidBodies[j].Markers, ptr, nBytes);
      ptr += nBytes;
      if(major >= 2) {
        // associated marker IDs
        nBytes = nRigidMarkers*sizeof(int);
        data.RigidBodies[j].MarkerIDs = (int*)malloc(nBytes);
        memcpy(data.RigidBodies[j].MarkerIDs, ptr, nBytes);
        ptr += nBytes;
        
        // associated marker sizes
        nBytes = nRigidMarkers*sizeof(float);
        data.RigidBodies[j].MarkerSizes = (float*)malloc(nBytes);
        memcpy(data.RigidBodies[j].MarkerSizes, ptr, nBytes);
        ptr += nBytes;

        // Mean marker error
        memcpy(&(data.RigidBodies[j].MeanError), ptr, 4); ptr += 4;
      } // end if major >= 2

      // 2.6 and later
      if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) {
        // params
        memcpy(&(data.RigidBodies[j].params), ptr, 2); ptr += 2;
        // Reference code        
        bool bTrackingValid = data.RigidBodies[j].params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
      }        
    } // next rigid body


    // TODO: move RigidBody, Skeleton, etc. parsing into functions, so much code dup = =            
    // skeletons (version 2.1 and later), ignored for now cos we don't use skeletons
    
    if( ((major == 2)&&(minor>0)) || (major>2)) {
      int nSkeletons = 0; memcpy(&nSkeletons, ptr, 4); ptr += 4;
      data.nSkeletons = nSkeletons;
      //printf("nSkeletons = %d\n", nSkeletons);
/*
      for (int i=0; i < nSkeletons; i++)  {
        // skeleton id
        memcpy(&(data.Skeletons[i].skeletonID), ptr, 4); ptr += 4;
        // # of rigid bodies (bones) in skeleton
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        data.Skeletons[i].nRigidBodies = nRigidBodies;
        for (int j=0; j < nRigidBodies; j++) {
          // rigid body pos/ori
          int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
          float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
          float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
          float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
          float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
          float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
          float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
          float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
          data.Skeletons[i].

          // associated marker positions
          int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
          printf("Marker Count: %d\n", nRigidMarkers);
          int nBytes = nRigidMarkers*3*sizeof(float);
          float* markerData = (float*)malloc(nBytes);
          memcpy(markerData, ptr, nBytes);
          ptr += nBytes;

          // associated marker IDs
          nBytes = nRigidMarkers*sizeof(int);
          int* markerIDs = (int*)malloc(nBytes);
          memcpy(markerIDs, ptr, nBytes);
          ptr += nBytes;

          // associated marker sizes
          nBytes = nRigidMarkers*sizeof(float);
          float* markerSizes = (float*)malloc(nBytes);
          memcpy(markerSizes, ptr, nBytes);
          ptr += nBytes;

          for(int k=0; k < nRigidMarkers; k++)
          {
            printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
          }

          // Mean marker error
          float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
          printf("Mean marker error: %3.2f\n", fError);

          // release resources
          if(markerIDs)
              free(markerIDs);
          if(markerSizes)
              free(markerSizes);
          if(markerData)
              free(markerData);

        } // next rigid body
      } // next skeleton
*/
    }

    // labeled markers (version 2.3 and later)
    if( ((major == 2)&&(minor>=3)) || (major>2)) {
      int nLabeledMarkers = 0;
      memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
      //printf("nLabeledMarkers = %d\n", nLabeledMarkers);
      data.nLabeledMarkers = nLabeledMarkers;
      for (int j=0; j < nLabeledMarkers; j++) {	
	int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
	float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
        float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
	float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
	float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;
        data.LabeledMarkers[j].ID = ID;
        data.LabeledMarkers[j].x = x;
        data.LabeledMarkers[j].y = y;
        data.LabeledMarkers[j].z = z;
        data.LabeledMarkers[j].size = size;
        // 2.6 and later
        if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) {
          // marker params
          short params = 0; memcpy(&params, ptr, 2); ptr += 2;
          data.LabeledMarkers[j].params = params;
          // Reference code
          bool bOccluded = params & 0x01;     // marker was not visible (occluded) in this frame
          bool bPCSolved = params & 0x02;     // position provided by point cloud solve
          bool bModelSolved = params & 0x04;  // position provided by model solve
    } } }

    // latency
    memcpy(&(data.fLatency), ptr, 4); ptr += 4;
    //printf("Latency =\t%f\n", data.fLatency);

    // timecode
    memcpy(&(data.Timecode), ptr, 4);	ptr += 4;
    memcpy(&(data.TimecodeSubframe), ptr, 4); ptr += 4;
    // Reference code to decode Timecodes
    char szTimecode[128] = "";
    TimecodeStringify(data.Timecode, data.TimecodeSubframe, szTimecode, 128);
    //printf("Timecode =\t%s\n", szTimecode);

    // timestamp
    // 2.7 and later - increased from single to double precision
    double timestamp = -1.0f;
    if( ((major == 2)&&(minor>=7)) || (major>2)) {
      memcpy(&timestamp, ptr, 8); ptr += 8;
    } else if ((major == 2) && (minor>=6)){
      float fTemp = 0.0f;
      memcpy(&fTemp, ptr, 4); ptr += 4;
      timestamp = (double)fTemp;
    }
    data.fTimestamp = timestamp;
    //printf("Timestamp =\t%f\n", timestamp);
    
    // frame params
    memcpy(&(data.params), ptr, 2); ptr += 2;
    // Reference code
    bool bIsRecording = data.params & 0x01;                  // 0x01 Motive is recording
    bool bTrackedModelsChanged = data.params & 0x02;         // 0x02 Actively tracked model list has changed
/*
    if (bIsRecording) 
      printf("bIsRecording\n");
    else 
      printf("Not bIsRecording\n");
    if (bTrackedModelsChanged) 
      printf("bTrackedModelsChanged\n");
    else 
      printf("Not bTrackedModelsChanged\n"); 
*/
    // end of data tag
    int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
  } // end if MessageID == NAT_FRAMEOFDATA (7)

  else if(MessageID == NAT_MODELDEF) { // Data Description (5)
    // number of datasets
    int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
    desc.nDataDescriptions = nDatasets;
    printf("nDataDescriptions: %d\n", nDatasets);

    for(int i=0; i < nDatasets; i++)
    {
      //printf("Dataset %d\n", i);

      int type = 0; memcpy(&type, ptr, 4); ptr += 4;
      desc.arrDataDescriptions[i].type = type;
      //printf("Type : %d\n", i, type);

      if(type == 0)   // markerset
      {
        if (desc.arrDataDescriptions[i].Data.MarkerSetDescription != NULL) {
        }
        desc.arrDataDescriptions[i].Data.MarkerSetDescription = 
          (sMarkerSetDescription *)malloc(sizeof(sMarkerSetDescription));        

        // name
        strcpy(desc.arrDataDescriptions[i].Data.MarkerSetDescription->szName, ptr);
        int nDataBytes = (int) strlen(desc.arrDataDescriptions[i].Data.MarkerSetDescription->szName) + 1;
        ptr += nDataBytes; 
        //printf("Markerset Name: %s\n", szName);

        // marker data
        int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
        desc.arrDataDescriptions[i].Data.MarkerSetDescription->nMarkers = nMarkers;
        //printf("Marker Count : %d\n", nMarkers);

        desc.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames = 
          (char **)malloc(nMarkers*sizeof(char *));

        for(int j=0; j < nMarkers; j++)
        {         
          desc.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames[j] = (char *)malloc(MAX_NAMELENGTH);
          strcpy(desc.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames[j], ptr);
          int nDataBytes = (int) strlen(desc.arrDataDescriptions[i].Data.MarkerSetDescription->szMarkerNames[j]) + 1;
          ptr += nDataBytes;
          //printf("Marker Name: %s\n", szName);
        }
      }
      else if(type ==1){   // rigid body
        desc.arrDataDescriptions[i].Data.RigidBodyDescription = 
          (sRigidBodyDescription *)malloc(sizeof(sRigidBodyDescription));  
        if(major >= 2)
        {
          // name
          strcpy(desc.arrDataDescriptions[i].Data.RigidBodyDescription->szName, ptr);
          int nDataBytes = (int) strlen(desc.arrDataDescriptions[i].Data.RigidBodyDescription->szName) + 1;
          ptr += nDataBytes; 
          printf("Name: %s\n", desc.arrDataDescriptions[i].Data.RigidBodyDescription->szName);
        }

        memcpy(&(desc.arrDataDescriptions[i].Data.RigidBodyDescription->ID), ptr, 4); ptr +=4;
        printf("ID : %d\n", desc.arrDataDescriptions[i].Data.RigidBodyDescription->ID);
     
        memcpy(&(desc.arrDataDescriptions[i].Data.RigidBodyDescription->parentID), ptr, 4); ptr +=4;
        printf("Parent ID : %d\n", desc.arrDataDescriptions[i].Data.RigidBodyDescription->parentID);
            
        memcpy(&(desc.arrDataDescriptions[i].Data.RigidBodyDescription->offsetx), ptr, 4); ptr +=4;
        //printf("X Offset : %3.2f\n", xoffset);

        memcpy(&(desc.arrDataDescriptions[i].Data.RigidBodyDescription->offsety), ptr, 4); ptr +=4;
        //printf("Y Offset : %3.2f\n", yoffset);

        memcpy(&(desc.arrDataDescriptions[i].Data.RigidBodyDescription->offsetz), ptr, 4); ptr +=4;
        //printf("Z Offset : %3.2f\n", zoffset);

      } else if(type == 2) {   // skeleton
        desc.arrDataDescriptions[i].Data.SkeletonDescription = 
          (sSkeletonDescription *)malloc(sizeof(sSkeletonDescription));  
        // name
        strcpy(desc.arrDataDescriptions[i].Data.SkeletonDescription->szName, ptr);
        int nDataBytes = (int) strlen(desc.arrDataDescriptions[i].Data.SkeletonDescription->szName) + 1;
        ptr += nDataBytes; 
        //printf("Name: %s\n", szName);

        memcpy(&(desc.arrDataDescriptions[i].Data.SkeletonDescription->skeletonID), ptr, 4); ptr +=4;
        //printf("ID : %d\n", ID);

        int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
        desc.arrDataDescriptions[i].Data.SkeletonDescription->nRigidBodies = nRigidBodies;
        //printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

        for(int j=0; j < nRigidBodies; j++)   {
          if(major >= 2)  {
            // RB name
            strcpy(desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].szName, ptr);
            int nDataBytes = 
              (int) strlen(desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].szName) + 1;
            ptr += nDataBytes; 
            //printf("Rigid Body Name: %s\n", szName);
          }

          memcpy(&desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].ID, ptr, 4); ptr +=4;
          //printf("RigidBody ID : %d\n", ID);

          memcpy(&desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].parentID, ptr, 4); ptr +=4;
          //printf("Parent ID : %d\n", parentID);

          memcpy(&desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsetx, ptr, 4); ptr +=4;
          //printf("X Offset : %3.2f\n", xoffset);

          memcpy(&desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsety, ptr, 4); ptr +=4;
          //printf("Y Offset : %3.2f\n", yoffset);

          memcpy(&desc.arrDataDescriptions[i].Data.SkeletonDescription->RigidBodies[j].offsetz, ptr, 4); ptr +=4;
          //printf("Z Offset : %3.2f\n", zoffset);
        }  // end for
      } // end if type == 2
    }   // next dataset
  //printf("End Packet\n-------------\n");
  }
  else {
      //printf("Unrecognized Packet Type.\n");
  }
}

int main (int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "natnet_data_node");
  ros::NodeHandle n("~");
  static tf::TransformBroadcaster br;

  // TODO: Get configuration from ROS parameter server  
  /*
  const char** mocap_model( DEFAULT_MOCAP_MODEL );
  if( n.hasParam( MOCAP_MODEL_KEY ) )  {    
    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )  {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
  } }

  RigidBodyMap published_rigid_bodies;

  if (n.hasParam(RIGID_BODIES_KEY)) {
    XmlRpc::XmlRpcValue body_list;
    n.getParam("rigid_bodies", body_list);
    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0) {
      XmlRpc::XmlRpcValue::iterator i;
      for (i = body_list.begin(); i != body_list.end(); ++i) {
        if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          PublishedRigidBody body(i->second);
          string id = (string&) (i->first);
          RigidBodyItem item(atoi(id.c_str()), body);

          std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
          if (!result.second)  {
            ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
  }}}}}
  */

  // create unix socket to read incoming data
  UdpMulticastSocket multicast_client_socket( DATA_PORT, MULTICAST_IP );
  ushort payload;
  sDataDescriptions desc;
  rosbag::Bag bag;
  bool isPreviouslyRecording = false;
  int numReceivedPacket = 0;
  int numBagSequence = 0;
  // Process mocap data until SIGINT
  while(ros::ok()) {
    bool packetread = false;
    int numBytes = 0;
    do {
      // Receive data from mocap device
      numBytes = multicast_client_socket.recv();
      if( numBytes > 0 ) {
        const char* buffer = multicast_client_socket.getBuffer();        
        numReceivedPacket = (numReceivedPacket + 1) % 200;
        if (numReceivedPacket == 0) { 
          printf("Data is streaming...\n");
        }
        // parse a data packet
        sFrameOfMocapData data;
        parseDataPacket(buffer, data, desc);
        bool bIsRecording = data.params & 0x01;
        if (bIsRecording && !isPreviouslyRecording) { 
          char bag_name[20];
          sprintf(bag_name, "test_traj_%03d.bag", ++numBagSequence);
          bag.open(bag_name, rosbag::bagmode::Write);
          printf("Recording %s\n", bag_name);
        } else if (!bIsRecording && isPreviouslyRecording) {
          printf("Stopped recording\n");
          bag.close();
        }
        // publish rigid body data to tf, if any is available
        // also start recording tf data into rosbag 
        broadcastRecordTfData(br, data, bag);
        // Free memory
        destroyDataPacket(data);
        packetread = true;
        isPreviouslyRecording = bIsRecording;
      } 
    } while( numBytes > 0 );

    // Don't try again immediately
    if( !packetread )
      usleep( 10 );

  } // end while ros::ok
  return 0;
}
