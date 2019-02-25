  detection_avaliable = 1;//test code
  //检测结果有效时进入点云处理
  if (detection_avaliable)
  {
    MatrixXf box_left;
    MatrixXf box_right;
    MatrixXf box_left_side;
    MatrixXf box_right_side;
    MatrixXf box_left_xyz;
    MatrixXf box_right_xyz;
    MatrixXf box_distance;
    int vaild_point_num = 0;
    int invaild_point_num = 0;
    //定义旋转矩阵
    Matrix2f rotation_matrix;
    rotation_matrix (0,0) = cos (box_theta);
    rotation_matrix (0,1) = -sin(box_theta);
    rotation_matrix (1,0) = sin (box_theta);
    rotation_matrix (1,1) = cos (box_theta);
    cout<<rotation_matrix<<endl;
    //为了计算box窄边的宽度，判断那边比较段
    if (box_height > box_width)
    {
      box_left_side.resize(2,box_height);
      box_right_side.resize(2,box_height);
      for(int i = 0; i < box_height; i ++)
      {
        //left x
        box_left_side(0, i) = box_x - box_width;
        //left y
        box_left_side(1, i) = box_y - int(box_height/2) + i;
        //right x
        box_right_side(0, i) = box_x + box_width;
        //right y
        box_right_side(1, i) = box_y - int(box_height/2) + i;
      }
    }
    else
    {
      box_left_side.resize(2,box_width);
      box_right_side.resize(2,box_width);
      for(int i = 0; i < box_width; i ++)
      {
        
        //left x
        box_left_side(0, i) = box_x - int(box_width/2) + i;
        //left y
        box_left_side(1, i) = box_y - box_height;
        //right x
        box_right_side(0, i) = box_x - int(box_width/2) + i;
        //right y
        box_right_side(1, i) = box_y + box_height;

        grab_theta = box_theta + float(M_PI/2);

        cout<<"i:"<<i<<endl;
      }
    }
    //和旋转矩阵相乘，得到旋转box的两个边的xy值
    box_left = rotation_matrix * box_left_side;
    box_right = rotation_matrix * box_right_side;
    cout<<"over1"<<endl;
    //循环读取对应像素点上的深度三维信息
    for(int i = 0; i < max(box_height,box_width); i++)
    {
      float leftPointXYZ[3];
      float rightPointXYZ[3];
      // pixelTo3DPoint(receivedPointCloud, box_left_side(0, i),box_left_side(0, i), leftPointXYZ);
      pixelTo3DPoint(receivedPointCloud, 1,1, leftPointXYZ);
      pixelTo3DPoint(receivedPointCloud, box_right_side(0, i),box_right_side(0, i), rightPointXYZ);
      cout<<"over1:"<<i<<endl;
      cout<<leftPointXYZ[0]<<leftPointXYZ[1]<<leftPointXYZ[2]<<endl;
      if (leftPointXYZ[0] < 20 and leftPointXYZ[0] > -20 and \
         leftPointXYZ[1] < 20 and leftPointXYZ[1] > -20 and  \
         leftPointXYZ[2] < 20 and leftPointXYZ[2] > -20 and \
         rightPointXYZ[0] < 20 and rightPointXYZ[0] > -20 and \
         rightPointXYZ[1] < 20 and rightPointXYZ[1] > -20 and \
         rightPointXYZ[2] < 20 and rightPointXYZ[2] > -20)
      {
        box_left_xyz(0,vaild_point_num) = leftPointXYZ[0];
        box_left_xyz(1,vaild_point_num) = leftPointXYZ[1];
        box_left_xyz(2,vaild_point_num) = leftPointXYZ[2];

        box_right_xyz(0,vaild_point_num) = rightPointXYZ[0];
        box_right_xyz(1,vaild_point_num) = rightPointXYZ[1];
        box_right_xyz(2,vaild_point_num) = rightPointXYZ[2];

        vaild_point_num = vaild_point_num + 1;
      }
      else
      {
        invaild_point_num = invaild_point_num + 1;
      }  
    }
    cout<<"vaild_point_num:"<<vaild_point_num<<endl;
    //求两点欧式距离
    box_distance = (box_left_xyz - box_right_xyz);
    box_distance.squaredNorm();
    box_distance.rowwise().sum();
    box_distance.cwiseSqrt();

    //标志位置0
    detection_avaliable = 0;
  }


      u = central_u - pix_unit + pix_unit * k;
    v = central_v - pix_unit;
    //每个大点中选择九个小点，两个循环分别遍历行列
    for (int i = depth_point_level; i > 0; i--)
    {
      for (int j = depth_point_level; j > 0; j--)
      {
        u = u - int(depth_point_level / 2) + i;
        v = v - int(depth_point_level / 2) + j;
        // get width and height of 2D point cloud data
        int width = receivedPointCloud.width;
        int height = receivedPointCloud.height;

        // Convert from u (column / width), v (row/height) to position in array
        // where X,Y,Z data starts
        int arrayPosition = v * receivedPointCloud.row_step + u * receivedPointCloud.point_step;

        // compute position in array where x,y,z data start
        int arrayPosX = arrayPosition + receivedPointCloud.fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + receivedPointCloud.fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + receivedPointCloud.fields[2].offset; // Z has an offset of 8

        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;

        memcpy(&X, &receivedPointCloud.data[arrayPosX], sizeof(float));
        memcpy(&Y, &receivedPointCloud.data[arrayPosY], sizeof(float));
        memcpy(&Z, &receivedPointCloud.data[arrayPosZ], sizeof(float));

        depthcloud->points.push_back(pcl::PointXYZ(X, Y, Z));
      }
    }


    int u = central_u - pix_unit + pix_unit * k;
    int v = central_v + pix_unit;
    for (int i = depth_point_level; i > 0; i--)
    {
      for (int j = depth_point_level; j > 0; j--)
      {
        // pixelTo3DPoint(inputCloud , u - int(level / 2) + i , v - int(level / 2) + j, pointXYZ);
        // pixelTo3DPoint(inputCloud , 1, 1, pointXYZ);
        u = u - int(depth_point_level / 2) + i;
        v = v - int(depth_point_level / 2) + j;
        // get width and height of 2D point cloud data
        int width = receivedPointCloud.width;
        int height = receivedPointCloud.height;

        // Convert from u (column / width), v (row/height) to position in array
        // where X,Y,Z data starts
        int arrayPosition = v * receivedPointCloud.row_step + u * receivedPointCloud.point_step;

        // compute position in array where x,y,z data start
        int arrayPosX = arrayPosition + receivedPointCloud.fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + receivedPointCloud.fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + receivedPointCloud.fields[2].offset; // Z has an offset of 8

        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;

        memcpy(&X, &receivedPointCloud.data[arrayPosX], sizeof(float));
        memcpy(&Y, &receivedPointCloud.data[arrayPosY], sizeof(float));
        memcpy(&Z, &receivedPointCloud.data[arrayPosZ], sizeof(float));

        depthcloud->points.push_back(pcl::PointXYZ(X, Y, Z));
      }
    }


      //处理左边点
  u = central_u - pix_unit;
  v = central_v;
  for (int i = depth_point_level; i > 0; i--)
  {
    for (int j = depth_point_level; j > 0; j--)
    {
      // pixelTo3DPoint(inputCloud , u - int(level / 2) + i , v - int(level / 2) + j, pointXYZ);
      // pixelTo3DPoint(inputCloud , 1, 1, pointXYZ);
      u = u - int(depth_point_level / 2) + i;
      v = v - int(depth_point_level / 2) + j;
      // get width and height of 2D point cloud data
      int width = receivedPointCloud.width;
      int height = receivedPointCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = v * receivedPointCloud.row_step + u * receivedPointCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + receivedPointCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + receivedPointCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + receivedPointCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &receivedPointCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &receivedPointCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &receivedPointCloud.data[arrayPosZ], sizeof(float));

      depthcloud->points.push_back(pcl::PointXYZ(X, Y, Z));
    }
  }
  //处理右边点
  u = central_u + pix_unit;
  v = central_v;
  for (int i = depth_point_level; i > 0; i--)
  {
    for (int j = depth_point_level; j > 0; j--)
    {
      // pixelTo3DPoint(inputCloud , u - int(level / 2) + i , v - int(level / 2) + j, pointXYZ);
      // pixelTo3DPoint(inputCloud , 1, 1, pointXYZ);
      u = u - int(depth_point_level / 2) + i;
      v = v - int(depth_point_level / 2) + j;
      // get width and height of 2D point cloud data
      int width = receivedPointCloud.width;
      int height = receivedPointCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = v * receivedPointCloud.row_step + u * receivedPointCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + receivedPointCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + receivedPointCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + receivedPointCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &receivedPointCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &receivedPointCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &receivedPointCloud.data[arrayPosZ], sizeof(float));

      depthcloud->points.push_back(pcl::PointXYZ(X, Y, Z));
    }
  }