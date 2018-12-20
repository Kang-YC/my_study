 rotation test


 for(int k=0;k<CloudKeyFramesOri[ind]->points.size();k++)
    {
      
      Vector3d ori, proj;

      //ori in body frame
      //proj in world frame

      ori<< CloudKeyFramesOri[ind]->points[k].z , CloudKeyFramesOri[ind]->points[k].x , CloudKeyFramesOri[ind]->points[k].y;
      
      proj<< CloudKeyFramesProj[ind]->points[k].z, CloudKeyFramesProj[ind]->points[k].x , CloudKeyFramesProj[ind]->points[k].y;

      // test the rotation
      if(k < 3 && ind == WINDOW_SIZE-1){
      double point[3] = {ori[0], ori[1], ori[2]};
      double q[4] = {para_Pose[ind][6],para_Pose[ind][3],para_Pose[ind][4],para_Pose[ind][5]};//!!!!!w x y z 
      double p[3];


      ceres::QuaternionRotatePoint( q, point, p);
      
        p[0] += para_Pose[ind][0];
        p[1] += para_Pose[ind][1];
        p[2] += para_Pose[ind][2];
      cout << "ori"<<p[0]<<" " <<p[1]<<" "<<p[2]<< endl;
      cout << "pro"<<proj[0]<<" " << proj[1]<<" "<<proj[2]<<endl;

      }
      


      int oriNum = LaserCloudOri->points.size();
   if (oriNum != 0)
   {
  float ctRoll = cos(roll);
  float stRoll = sin(roll);

  float ctPitch = cos(pitch);
  float stPitch = sin(pitch);

  float ctYaw = cos(yaw);
  float stYaw = sin(yaw);

  float tInX = x;
  float tInY = y;
  float tInZ = z;
    for(int i = 0;i<3;i++){

            PointType pointFrom = LaserCloudOri->points[i];
            PointType pointTo = LaserCloudOri -> points[i];
            PointType pointProj = LaserCloudProj -> points[i];

            float x1 = ctRoll * pointFrom.x - stRoll * pointFrom.y;
            float y1 = stRoll * pointFrom.x + ctRoll* pointFrom.y;
            float z1 = pointFrom.z;

            float x2 = x1;
            float y2 = ctPitch * y1 - stPitch * z1;
            float z2 = stPitch * y1 + ctPitch* z1;

            pointTo.x = ctYaw * x2 + stYaw * z2 + tInY;
            pointTo.y = y2 + tInZ;
            pointTo.z = -stYaw * x2 + ctYaw * z2 + tInX;

            ROS_DEBUG("op to  %f %f %f ",pointTo.x, pointTo.y,pointTo.z);
            ROS_DEBUG("op pro %f %f %f ",pointProj.x, pointProj.y,pointProj.z);

      Vector3d ori, proj;
      ori<< pointFrom.z , pointFrom.x , pointFrom.y;
      
      proj<< pointProj.z, pointProj.x , pointProj.y;

      double point[3] = {ori[0], ori[1], ori[2]};
      cout<<q.x()<<endl;
      double q1[4] = {q.w(),q.x(),q.y(),q.z()};//!!!!!w x y z 
      double p[3];
      ceres::QuaternionRotatePoint( q1, point, p);
        p[0] += tInX;
        p[1] += tInY;
        p[2] += tInZ;
         cout << "ori"<<p[0]<<" " <<p[1]<<" "<<p[2]<< endl;
      cout << "pro"<<proj[0]<<" " << proj[1]<<" "<<proj[2]<<endl;


        }

   }
