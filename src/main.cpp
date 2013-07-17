#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <navigationISL/neighborInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <ctime>
#include <sstream>

#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>


geometry_msgs::PoseWithCovarianceStamped robotPose;

#define numOfRobots 5
const int maxIteration = 11000;

double poses[maxIteration][3];
int numOfPoses;
int robotID;

bool startPublishingPose = false;

// Received start info
void startInfoCallback(navigationISL::neighborInfo neighborInfo)
{
    QString str = QString::fromStdString(neighborInfo.name);

    if(str == "start")
    {
        startPublishingPose = true;
    }

    return;
}


// Reads the config file
bool readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {

        robotID = result["robotID"].toInt();

        qDebug()<<result["robotID"].toString();

    }
    file.close();
    return true;

}

// read simulated poses
bool readPoses(QString fileName)
{
    QFile file(fileName);

    if(!file.open(QFile::ReadOnly))
    {
        return false;
    }

    QTextStream stream(&file);
    int i  = 0;
    while(!stream.atEnd())
    {

        QString str = stream.readLine();

        QStringList posestr = str.split(" ");

        if (i<maxIteration)
        {
            poses[i][1] = posestr.at(2*robotID-2).toDouble()/100;
            poses[i][2] = posestr.at(2*robotID-1).toDouble()/100;
            //qDebug()<<  poses[i][1]<<" "<<poses[i][2];

            i = i + 1;
        }
    }



    numOfPoses = i;

    std::cout << numOfPoses;

    file.close();

    return true;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amclfakeISL");

    ros::NodeHandle n;

    ros::Publisher amclPosePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1000);

    ros::Subscriber startInfoSubscriber = n.subscribe("communicationISL/neighborInfo",5, startInfoCallback);

    ros::Rate loop_rate(0.2);

    QString path = QDir::homePath();
    path.append("/fuerte_workspace/sandbox/configISL.json");


    if(!readConfigFile(path)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        return 0;
    }


    path = QDir::homePath();
    path.append("/fuerte_workspace/sandbox/poses.txt");
    if(!readPoses(path)){

        qDebug()<< "Read Pose File Failed!!!";

        ros::shutdown();

        return 0;
    }

    /*
  FILE *fp;
  fp = fopen("poses.txt","r");
  if(fp==NULL)
  {
      std::cout <<"Error - could not open poses.txt";
      return 0;
  }
  else
  {
      float x[numOfRobots+1], y[numOfRobots+1];
      int i = 0;
      while (!feof(fp))
      {
          fscanf(fp, "%f %f %f %f %f %f %f %f %f %f", &x[1], &y[1], &x[2], &y[2], &x[3], &y[3], &x[4], &y[4], &x[5], &y[5]);
          //poses[i][0] = t;
          poses[i][1] = x[robotID]/100;
          poses[i][2] = y[robotID]/100;
          //qDebug()<<  poses[i][1]<<" "<<poses[i][2];

          i = i + 1;
      }

      numOfPoses = i;
  }

  std::cout << numOfPoses;
*/


    int timeIndx = 0;
    while (ros::ok())
    {
        if (startPublishingPose)
        {
            if (timeIndx<numOfPoses)
            {

                robotPose.pose.pose.position.x = poses[timeIndx][1]; //in meters

                robotPose.pose.pose.position.y = poses[timeIndx][2]; //in meters

                robotPose.pose.pose.position.z = 0.1;

                robotPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

                amclPosePublisher.publish(robotPose);
            }
            else
            {
                return 0;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();

        timeIndx = timeIndx + 1;

    }


    return 0;
}


