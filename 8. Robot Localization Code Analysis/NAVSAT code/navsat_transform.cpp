namespace RobotLocalization
{ 
  //navsat_transform의 NavSatTransform 클래스의 NavSatTransform의 값들 초기화
  //클래스 안에 선언된 생성자의 초기화를 목적으로 선언된 생성자
  NavSatTransform::NavSatTransform(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    //멤버 이니셜라이저
    broadcast_cartesian_transform_(false),
    broadcast_cartesian_transform_as_parent_frame_(false),
    gps_updated_(false),
    has_transform_gps_(false),
    has_transform_imu_(false),
    has_transform_odom_(false),
    odom_updated_(false),
    publish_gps_(false),
    transform_good_(false),
    use_manual_datum_(false),
    use_odometry_yaw_(false),
    use_local_cartesian_(false),
    zero_altitude_(false),
    magnetic_declination_(0.0),
    yaw_offset_(0.0),
    base_link_frame_id_("base_link"),
    gps_frame_id_(""),
    utm_zone_(0),
    world_frame_id_("odom"),
    transform_timeout_(ros::Duration(0)),
    tf_listener_(tf_buffer_)
  {
    ROS_INFO("Waiting for valid clock time...");
    ros::Time::waitForValid();
    ROS_INFO("Valid clock time received. Starting node.");

    //최신 GPS/UTM/LocalCartesian/odom 데이터에 대한 공분산 사이즈 다시
    latest_cartesian_covariance_.resize(POSE_SIZE, POSE_SIZE);
    latest_odom_covariance_.resize(POSE_SIZE, POSE_SIZE);

    //주파수, 딜레이, 변환 타임아웃 선언 및 초기화
    double frequency;
    double delay = 0.0;
    double transform_timeout = 0.0;

    // Load the parameters we need
    // 필요한 parameter 로드 (private parameter로 접근)
    // param : getparam과 비슷 but, parameter가 검색할 수 없는 경우 기본값을 지정 가능
    nh_priv.getParam("magnetic_declination_radians", magnetic_declination_);
    nh_priv.param("yaw_offset", yaw_offset_, 0.0);
    nh_priv.param("broadcast_cartesian_transform", broadcast_cartesian_transform_, false);
    nh_priv.param("broadcast_cartesian_transform_as_parent_frame",
                  broadcast_cartesian_transform_as_parent_frame_, false);
    nh_priv.param("zero_altitude", zero_altitude_, false);
    nh_priv.param("publish_filtered_gps", publish_gps_, false);
    nh_priv.param("use_odometry_yaw", use_odometry_yaw_, false);
    nh_priv.param("wait_for_datum", use_manual_datum_, false);
    nh_priv.param("use_local_cartesian", use_local_cartesian_, false);
    nh_priv.param("frequency", frequency, 10.0);
    nh_priv.param("delay", delay, 0.0);
    nh_priv.param("transform_timeout", transform_timeout, 0.0);
    nh_priv.param("cartesian_frame_id", cartesian_frame_id_, std::string(use_local_cartesian_ ? "local_enu" : "utm"));
    transform_timeout_.fromSec(transform_timeout);

    // Check for deprecated parameters
    // 더 이상 사용되지 않는 parameter 확인
    if (nh_priv.getParam("broadcast_utm_transform", broadcast_cartesian_transform_))
    {
      ROS_WARN("navsat_transform, Parameter 'broadcast_utm_transform' has been deprecated. Please use"
               "'broadcast_cartesian_transform' instead.");
    }
    if (nh_priv.getParam("broadcast_utm_transform_as_parent_frame", broadcast_cartesian_transform_as_parent_frame_))
    {
      ROS_WARN("navsat_transform, Parameter 'broadcast_utm_transform_as_parent_frame' has been deprecated. Please use"
               "'broadcast_cartesian_transform_as_parent_frame' instead.");
    }

    // Check if tf warnings should be suppressed
    // tf 경고를 억제해야 하는지 확인 nh.getparam은 parameter server에서 값을 가져옴
    nh.getParam("/silent_tf_failure", tf_silent_failure_);

    // Subscribe to the messages and services we need
    // 필요한 메시지와 서비스를 구독 > service : 요청시 작동
    // datum=srv이름, &~=기능을 할 함수 
    datum_srv_ = nh.advertiseService("datum", &NavSatTransform::datumCallback, this);

    to_ll_srv_ = nh.advertiseService("toLL", &NavSatTransform::toLLCallback, this);
    from_ll_srv_ = nh.advertiseService("fromLL", &NavSatTransform::fromLLCallback, this);
    set_utm_zone_srv_ = nh.advertiseService("setUTMZone", &NavSatTransform::setUTMZoneCallback, this);

    // 만약, 첫 번째 GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 가져오고 datum이라는 parameter를 확인하면
    // hasparam : parameter의 존재를 확인할 수 있음
    if (use_manual_datum_ && nh_priv.hasParam("datum"))
    { 
      //XmlRpc:ROS launch file에서 yaml file을 load해주는 편리한 도구
      XmlRpc::XmlRpcValue datum_config;

      //예외 발생에 대한 검사의 범위를 지정
      try
      {
        double datum_lat;
        double datum_lon;
        double datum_yaw;

        nh_priv.getParam("datum", datum_config);

        // Handle datum specification. Users should always specify a baseLinkFrameId_ in the
        // datum config, but we had a release where it wasn't used, so we'll maintain compatibility.
        // 데이텀 사양을 처리
        // 사용자는 항상 데이텀 구성에서 baseLinkFrameId_를 지정해야 하지만 사용되지 않은 릴리스가 있으므로 호환성을 유지

        //assert함수를 사용하면 어디에 에러가 났는지 알려줌
        //datum_config의 타입이 typearray와 같다면, 에러 알려줘
        //datum_config의 사이즈가 3보다 크거나 같다면, 에러 알려줘
        ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(datum_config.size() >= 3);

        if (datum_config.size() > 3)
        {
          //더 이상 사용되지 않는 데이텀 매개변수 구성이 감지
          //처음 세 개의 매개변수 "(위도, 경도, yaw)만 사용되며, frame_ids는 odometry 및 navsat 입력에서 파생
          ROS_WARN_STREAM("Deprecated datum parameter configuration detected. Only the first three parameters "
              "(latitude, longitude, yaw) will be used. frame_ids will be derived from odometry and navsat inputs.");
        }

        //ostringstream을 사용해서 문자열 format을 조합하여 저장
        std::ostringstream ostr;
        //setprecision을 사용해서 부동 소수점에 대한 사용자 지정 정밀도 설정(20까지)
        ostr << std::setprecision(20) << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
        //istringstream을 사용해서 문자열 포맷을 parsing 
        std::istringstream istr(ostr.str());
        //읽은 datum_config[0~2]를 하나씩 쪼개서 datum_lat, lon, yaw로 parsing
        istr >> datum_lat >> datum_lon >> datum_yaw;

        // Try to resolve tf_prefix
        // tf_prefix 해결 시도
        std::string tf_prefix = "";
        std::string tf_prefix_path = "";

        //"tf_prefix"파라미터 찾으면 tf_prefix_path로 반환, path를 얻으면 tf_prefix로 parameter을 로드해라
        if (nh_priv.searchParam("tf_prefix", tf_prefix_path))
        {
          nh_priv.getParam(tf_prefix_path, tf_prefix);
        }

        // Append the tf prefix in a tf2-friendly manner
        // tf2 친화적인 방식으로 tf 접두사 추가
        // ❓️ FilterUtilities의 appendprefix를 사용하여 첫번째 변수를 두번째 변수의 접두사로 해라 ❓️
        FilterUtilities::appendPrefix(tf_prefix, world_frame_id_);
        FilterUtilities::appendPrefix(tf_prefix, base_link_frame_id_);

        //재정의 후 초기화
        robot_localization::SetDatum::Request request;
        request.geo_pose.position.latitude = datum_lat;
        request.geo_pose.position.longitude = datum_lon;
        request.geo_pose.position.altitude = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, datum_yaw);
        request.geo_pose.orientation = tf2::toMsg(quat);
        robot_localization::SetDatum::Response response;
        datumCallback(request, response);
      }

      //try블록에서 발생한 예외를 처리하는 코드가 담김
      catch (XmlRpc::XmlRpcException &e)
      {
        //process_noise_covariance에 대한 센서 구성을 읽는 동안 오류가 발생
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << datum_config.getType() << ")");
      }
    }

    //odom, gps 구독
    odom_sub_ = nh.subscribe("odometry/filtered", 1, &NavSatTransform::odomCallback, this);
    gps_sub_  = nh.subscribe("gps/fix", 1, &NavSatTransform::gpsFixCallback, this);

    //만약 첫 번째 GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 가져오고, odom or IMU 소스에서 yaw을 가져오면
    if (!use_odometry_yaw_ && !use_manual_datum_)
    { 
      //imu 구독
      imu_sub_ = nh.subscribe("imu/data", 1, &NavSatTransform::imuCallback, this);
    }

    //GPS 데이터용 게시
    gps_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry/gps", 10);

    //만약 필터링된 GPS 메시지 게시
    if (publish_gps_)
    { 
      //필터링된 gps 게시
      filtered_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps/filtered", 10);
    }

    // Sleep for the parameterized amount of time, to give
    // other nodes time to start up (not always necessary)
    // 다른 노드가 시작할 시간을 주기 위해 매개변수화된 시간 동안 sleep(항상 필요한 것은 아님)
    ros::Duration start_delay(delay);
    start_delay.sleep();

    // 주기적 업데이트를 호출하는 타이머
    periodicUpdateTimer_ = nh.createTimer(ros::Duration(1./frequency), &NavSatTransform::periodicUpdate, this);
  }

  //Navsattransform class의 소멸자를 초기화해주기 위해 선언
  NavSatTransform::~NavSatTransform()
  {
  }

//  void NavSatTransform::run()
  //periodicupdate함수 정의 : 주기적 업데이트를 위해 호출되는 콜백 함수
  void NavSatTransform::periodicUpdate(const ros::TimerEvent& event)
  { 
    //좋은 헤딩을 계산을 안했더라면
    if (!transform_good_)
    { 
      //UTM 프레임에서 Odom 프레임으로의 변환을 계산 > computetransform 계산 함수는 아래에 있음
      computeTransform();

      // 만약 좋은 헤딩을 계산하고 odom/imu에서 yaw 안 가져오고 첫 번째 GPS 메시지 또는 set_datum에서 데이터를 안가져오면
      if (transform_good_ && !use_odometry_yaw_ && !use_manual_datum_)
      { 
        // 즉 좋은 헤딩 계산했는데 imu가 필요한 값들이 없으면
        // Once we have the transform, we don't need the IMU
        // 변환이 있으면 IMU가 필요하지 않음
        imu_sub_.shutdown();
      }
    }
    //좋은 헤딩을  계산했으면
    else
    { 
      //gps_odom 준비할 odom 측정 메시지를 output
      nav_msgs::Odometry gps_odom;
      //만약 전송하기 전에 GPS odom 메시지를 준비를 했으면
      if (prepareGpsOdometry(gps_odom))
      {
        //GPS 데이터용 게시자를 게시해라
        gps_odom_pub_.publish(gps_odom);
      }

      //만약 gps를 게시했다면
      if (publish_gps_)
      {
        //odom_gps을 msg출력해라
        sensor_msgs::NavSatFix odom_gps;
        //만약 필터링된 gps를 준비했다면
        if (prepareFilteredGps(odom_gps))
        { 
          //필터링된 GPS 데이터 게시자를 게시해라
          filtered_gps_pub_.publish(odom_gps);
        }
      }
    }
  }

  //변환 계산 함수
  void NavSatTransform::computeTransform()
  {
    // Only do this if:
    // 1. We haven't computed the odom_frame->cartesian_frame transform before
    // 2. We've received the data we need
    // 다음과 같은 경우에만 이 작업을 수행
    // 1. 이전에 odom_frame->cartesian_frame 변환을 계산하지 않았을 때
    // 2. 필요한 데이터를 받았을 때

    // 만약, 변환이 잘 안됐고(좋은 헤딩 못가짐), 사용가능한 odom/gps/imu 수신했으면
    // odom > 데카르트 변환 됐으면 당연히 필요 없쥐
    if (!transform_good_ &&
        has_transform_odom_ &&
        has_transform_gps_ &&
        has_transform_imu_)
    {
      // The cartesian pose we have is given at the location of the GPS sensor on the robot. We need to get the
      // cartesian pose of the robot's origin.
      // 데카르트 자세는 로봇의 GPS 센서 위치에 주어지므로 로봇 원점의 데카르트 자세를 가져와야 함

      // tf2::로봇 원점의 데카르트 자세 (=보정된 데카르트 자세)
      tf2::Transform transform_cartesian_pose_corrected;
      // 만약 첫 번째 GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 가져오지 않으면
      if (!use_manual_datum_)
      { 
        // 데카르트 좌표계에서 navsat 센서의 포즈가 주어지면 차량의 중심에서 오프셋을 제거하고 해당 중심의 데카르트 좌표 포즈를 반환
        // 첫번째 변수가 gps_odom_pose, 두번째 robot_odom_pose, 세번째 변환 시간
        getRobotOriginCartesianPose(transform_cartesian_pose_, transform_cartesian_pose_corrected, ros::Time(0));
      }
      else
      { 
        //로봇 원점의 데카르트 자세 = 로봇 GPS센서 위치
        transform_cartesian_pose_corrected = transform_cartesian_pose_;
      }

      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      // 쿼터니안 형식의 IMU의 현재 RPY 값을 가져오며, raw 값이 필요합니다(어쨌든 요(yaw)의 경우)
      // 쿼터니안 x,y,z,w 계산 어렵기 때문에 r,p,y로 계산후 쿼터니안으로 변환하면 쉬움
      tf2::Matrix3x3 mat(transform_orientation_);

      // Convert to RPY
      // RPY로 변환
      double imu_roll;
      double imu_pitch;
      double imu_yaw;
      mat.getRPY(imu_roll, imu_pitch, imu_yaw);

      /* The IMU's heading was likely originally reported w.r.t. magnetic north.
       * However, all the nodes in robot_localization assume that orientation data,
       * including that reported by IMUs, is reported in an ENU frame, with a 0 yaw
       * value being reported when facing east and increasing counter-clockwise (i.e.,
       * towards north). To make the world frame ENU aligned, where X is east
       * and Y is north, we have to take into account three additional considerations:
       *   1. The IMU may have its non-ENU frame data transformed to ENU, but there's
       *      a possibility that its data has not been corrected for magnetic
       *      declination. We need to account for this. A positive magnetic
       *      declination is counter-clockwise in an ENU frame. Therefore, if
       *      we have a magnetic declination of N radians, then when the sensor
       *      is facing a heading of N, it reports 0. Therefore, we need to add
       *      the declination angle.
       *   2. To account for any other offsets that may not be accounted for by the
       *      IMU driver or any interim processing node, we expose a yaw offset that
       *      lets users work with navsat_transform_node.
       *   3. UTM grid isn't aligned with True East\North. To account for the difference
       *      we need to add meridian convergence angle when using UTM. This value will be
       *      0.0 when use_local_cartesian is TRUE.
       */
      /* IMU의 표제는 원래 w.r.t. 자기 북쪽
       * 그러나 robot_localization의 모든 노드는 IMU에서 보고한 것을 포함하여 방향 데이터가 ENU 프레임에서 보고되고 동쪽을 향하고
       * 시계 반대 방향(즉, 북쪽을 향함)으로 증가할 때 보고되는 0 요 값과 함께 가정
       * X가 동쪽이고 Y가 북쪽인 세계 프레임 ENU를 정렬하려면 세 가지 추가 고려 사항을 고려
        * 1. IMU는 비 ENU 프레임 데이터를 ENU로 변환할 수 있지만 데이터가 magnetic declination에 대해 수정되지 않았을 가능성이 있음
        *  ㄴ 북반구를 기준으로 지구 상의 현재 위치에서 진북극(지리상의 북극점) 방향과 자기북극 방향(나침반의 빨간 바늘이 가리키는 방향) 사이의 각도
        * 양의 magnetic declination은 ENU 프레임에서 시계 반대 방향
        * 따라서 N 라디안의 magnetic declination이 있는 경우 센서가 N 방향을 향하고 있을 때 declination을 보고하므로 따라서 편각을 추가
        * 2. IMU 드라이버 또는 중간 처리 노드에서 설명할 수 없는 다른 오프셋을 설명하기 위해 
        * 사용자가 navsat_transform_node로 작업할 수 있도록 하는 yaw 오프셋을 노출
        * 3. UTM 그리드가 True East\North와 정렬되지 않음
        * 차이를 설명하기 위해 UTM을 사용할 때 자오선 수렴 각도를 추가
        * 이 값은 use_local_cartesian이 TRUE일 때 0.0
        */
      
      // 따라서 imu yaw = imu yaw + (magnetic declination + yaw 오프셋 + utm 자오선 수렴)
      imu_yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);

      // ROS 문자열 사용 1. magnetic declination 2. 유저가 명시한 offset 3. 자오선 수렴 4. 현재 변환 헤딩 요소
      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magnetic_declination_ <<
                      ", user-specified offset of " << yaw_offset_ <<
                      " and meridian convergence of " << utm_meridian_convergence_ << "." <<
                      " Transform heading factor is now " << imu_yaw);

      // Convert to tf-friendly structures
      // tf 친화적인 구조로 변환 > 쿼터니안 형식
      tf2::Quaternion imu_quat;
      imu_quat.setRPY(0.0, 0.0, imu_yaw);

      // The transform order will be orig_odom_pos * orig_cartesian_pos_inverse * cur_cartesian_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      // 변환 순서는 orig_odom_pos * orig_cartesian_pos_inverse * cur_cartesian_pos
      // 이런 식으로 하면 첫 번째 GPS 메시지를 받을 때 0이 아닌 주행 거리 측정 위치에 대처 가능

      // 오리엔테이션인 데카르트 포즈를 변환
      tf2::Transform cartesian_pose_with_orientation;
      // 보정된 데카르트 포즈(=로봇 원점 포즈)의 origin을 오리엔테이션 데카르트 포즈의 origin으로 설정
      cartesian_pose_with_orientation.setOrigin(transform_cartesian_pose_corrected.getOrigin());
      // 쿼터니안 값을 가지는 imu를 오리엔테이션 데카르트 포즈의 new 로테이션으로 가짐
      cartesian_pose_with_orientation.setRotation(imu_quat);

      // Remove roll and pitch from odometry pose
      // 오도메트리 포즈에서 롤과 피치 제거
      // Must be done because roll and pitch is removed from cartesian_pose_with_orientation
      // cartesian_pose_with_orientation에서 롤과 피치가 제거되었으므로 반드시 수행
      //  ㄴ 왜냐면 imu_quat.setRPY(0.0, 0.0, imu_yaw) 라서
      double odom_roll, odom_pitch, odom_yaw;
      // 최신 IMU 오리엔테이션(transform_world_pose_)을 3x3행렬로 나타내고 odom_roll,pitch,yaw를 RPY로 갖고, new 로테이션을 함
      tf2::Matrix3x3(transform_world_pose_.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
      // odom_quat을 쿼터니언으로 tf
      tf2::Quaternion odom_quat;
      // odom_quat은 RPY를 0 0 odom yaw로 세팅
      odom_quat.setRPY(0.0, 0.0, odom_yaw);
      // 최신 IMU 오리엔테이션(transform_world_pose_)를 transform_world_pose_yaw_only로 변환해라
      tf2::Transform transform_world_pose_yaw_only(transform_world_pose_);
      // odom_quat은 transform_world_pose_yaw_only의 new 로테이션으로 세팅
      transform_world_pose_yaw_only.setRotation(odom_quat);

      // 데카르트 월드 변환 = odom_quat X 쿼터니안 값을 가지는 imu의 inverse
      cartesian_world_transform_.mult(transform_world_pose_yaw_only, cartesian_pose_with_orientation.inverse());

      cartesian_world_trans_inverse_ = cartesian_world_transform_.inverse();

      // ROS 문자열 출력
      ROS_INFO_STREAM("Transform world frame pose is: " << transform_world_pose_); // 최신 imu 오리엔테이션
      ROS_INFO_STREAM("World frame->cartesian transform is " << cartesian_world_transform_); // odom quatximu quat(T)

      // 좋은 헤딩 여부 = true
      transform_good_ = true;

      // Send out the (static) Cartesian transform in case anyone else would like to use it.
      // 다른 사람이 사용하려는 경우에 대비하여 (정적) 데카르트 변환을 보냄

      // 데카르트 변환을 브로드캐스트 한다면
      if (broadcast_cartesian_transform_)
      { 
        // geometry_msgs::TransformStamped > 좌표 프레임 header.frame_id에서 좌표 프레임 child_frame_id로의 변환
        geometry_msgs::TransformStamped cartesian_transform_stamped;
        cartesian_transform_stamped.header.stamp = ros::Time::now();
        // broadcast_cartesian_transform_as_parent_frame : 데카르트 변환을 부모 프레임으로 브로드캐스트할지 여부, 기본값은 자식 프레임
        cartesian_transform_stamped.header.frame_id = (broadcast_cartesian_transform_as_parent_frame_ ?
                                                       cartesian_frame_id_ : world_frame_id_);
        cartesian_transform_stamped.child_frame_id = (broadcast_cartesian_transform_as_parent_frame_ ?
                                                      world_frame_id_ : cartesian_frame_id_);
        cartesian_transform_stamped.transform = (broadcast_cartesian_transform_as_parent_frame_ ?
                                             tf2::toMsg(cartesian_world_trans_inverse_) :
                                             tf2::toMsg(cartesian_world_transform_));
        // zero_altitude_ : 이 매개변수가 true이면 변환된 GPS 주행 거리 측정 메시지의 고도에 대해 항상 0을 보고                                     
        cartesian_transform_stamped.transform.translation.z = (zero_altitude_ ?
                                                           0.0 : cartesian_transform_stamped.transform.translation.z);
        cartesian_broadcaster_.sendTransform(cartesian_transform_stamped);
      }
    }
  }

  // 데이터 서비스에 대한 콜백 (request & response)
  bool NavSatTransform::datumCallback(robot_localization::SetDatum::Request& request,
                                      robot_localization::SetDatum::Response&)
  {
    // If we get a service call with a manual datum, even if we already computed the transform using the robot's
    // initial pose, then we want to assume that we are using a datum from now on, and we want other methods to
    // not attempt to transform the values we are specifying here.
    // 수동 데이텀으로 서비스 요청을 받으면 로봇의 초기 포즈를 사용하여 변환을 이미 계산했더라도
    // 우리는 지금부터 데이터를 사용하고 있다고 가정하고 여기에서 지정하는 값을 변환하려고 시도하지 않는 다른 방법을 원함

    // 첫 번째 GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 가져옴
    use_manual_datum_ = true;

    // 좋은 헤딩을 계산안함
    transform_good_ = false;

    // 단일 변수를 동적으로 할당하려면, new 연산자 사용 (메모리 관리 때문에 사용)
    // 할당된 메모리에 나중에 접근할 수 있도록 반환 값을 자체 포인터 변수에 할당
    // new : heap에 메모리 만들고 그 주소를 리턴 / delete : 내가 가리키는 주소 메모리 해제(필수 why? 메모리 누수 생김)
    sensor_msgs::NavSatFix *fix = new sensor_msgs::NavSatFix();
    // 포인터를 역참조(dereference)하여 메모리에 접근
    fix->latitude = request.geo_pose.position.latitude;     //위도 요청
    fix->longitude = request.geo_pose.position.longitude;   //경도 요청
    fix->altitude = request.geo_pose.position.altitude;     //고도 요청
    fix->header.stamp = ros::Time::now();
    fix->position_covariance[0] = 0.1;                      //❓️ position 공분산 왜 9개인지 모르겠음 ❓️
    fix->position_covariance[4] = 0.1;
    fix->position_covariance[8] = 0.1;
    fix->position_covariance_type = sensor_msgs::NavSatStatus::STATUS_FIX;  //position 공분산 타입 설정
    sensor_msgs::NavSatFixConstPtr fix_ptr(fix);
    //! 변환을 계산하는 데 사용할 GPS 데이터를 설정하는 데 사용
    //! input : msg > 변환에 사용할 NavSatFix 메시지
    void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg);

    // fix_ptr을 받아 변환을 계산하여 GPS 데이터 설정
    setTransformGps(fix_ptr);

    // odom 동적 메모리 할당
    nav_msgs::Odometry *odom = new nav_msgs::Odometry();
    odom->pose.pose.orientation.x = 0;
    odom->pose.pose.orientation.y = 0;
    odom->pose.pose.orientation.z = 0;
    odom->pose.pose.orientation.w = 1;
    odom->pose.pose.position.x = 0;
    odom->pose.pose.position.y = 0;
    odom->pose.pose.position.z = 0;
    odom->header.frame_id = world_frame_id_;
    odom->child_frame_id = base_link_frame_id_;
    nav_msgs::OdometryConstPtr odom_ptr(odom);
    setTransformOdometry(odom_ptr);

    // imu 동적 메모리 할당
    sensor_msgs::Imu *imu = new sensor_msgs::Imu();
    imu->orientation = request.geo_pose.orientation;
    imu->header.frame_id = base_link_frame_id_;
    sensor_msgs::ImuConstPtr imu_ptr(imu);
    imuCallback(imu_ptr);

    return true;
  }

  // toLLCallback : 위도, 경도 서비스에 대한 콜백
  bool NavSatTransform::toLLCallback(robot_localization::ToLL::Request& request,
                                     robot_localization::ToLL::Response& response)
  { 
    // 좋은 헤딩을 계산하지 않았다면
    if (!transform_good_)
    {
      // error msg 출력 및 false 반환
      ROS_ERROR("No transform available (yet)");
      return false;
    }
    // 3D 점과 벡터를 나타내는 point변수
    tf2::Vector3 point;
    // fromMsg(A,B) A:ROSmsg B:변환할 객체 : Ros type A를 Another type B로 변환
    tf2::fromMsg(request.map_point, point);
    //mapToLL 함수는 아래에 있음
    mapToLL(point, response.ll_point.latitude, response.ll_point.longitude, response.ll_point.altitude);

    return true;
  }

  // fromLLCallback : 위도, 경도로 부터의 서비스에 대한 콜백
  bool NavSatTransform::fromLLCallback(robot_localization::FromLL::Request& request,
                                       robot_localization::FromLL::Response& response)
  { 
    //toLL의 mapToLL함수에서 받은 변수들을 여기서 재정의
    double altitude = request.ll_point.altitude;
    double longitude = request.ll_point.longitude;
    double latitude = request.ll_point.latitude;

    // 데카르트 포즈 변환
    tf2::Transform cartesian_pose;

    // 데카르트 포즈에서 사용할 변수들 정의
    double cartesian_x;
    double cartesian_y;
    double cartesian_z;

    // 로컬 데카르트(탄젠트 평면 ENU) 또는 UTM 좌표를 데카르트 좌표로 사용한다면
    if (use_local_cartesian_)
    { 
      // gps_local_cartesian_ : GPS 원점 주변의 로컬 데카르트 투영법
      // 당연한게 UTM 좌표는 3차원 데카르트 2차원이라서 projection 해야됨
      gps_local_cartesian_.Forward(latitude, longitude, altitude, cartesian_x, cartesian_y, cartesian_z);
    }
    // UTM 좌표를 데카르트 좌표로 사용하지 않는다면
    else
    {
      int zone_tmp;
      bool nortp_tmp;
      // 예외상황 배제 : GeographicLib::UTMUPS를 해라
      try
      {
        GeographicLib::UTMUPS::Forward(latitude, longitude, zone_tmp, nortp_tmp, cartesian_x, cartesian_y, utm_zone_);
      }
      // 예외상황 : 
      catch (const GeographicLib::GeographicErr& e)
      { 
        // ROS_ERROR_STREAM_THROTTLE(rate, name)
        // printf 스타일 형식을 사용하여 특정 인쇄 속도로 제한된 지정된 상세 수준에서 지정된 명명된 로거에 기록
        // what() 은 const char을 반환하는 가상 멤버 함수 즉, 위의 const GeographicLib::GeographicErr& e 반환
        ROS_ERROR_STREAM_THROTTLE(1.0, e.what());
        return false;
      }
    }

    // 데카르트 좌표 origin설정 (벡터 형식의 데카르트 x,y,고도)
    cartesian_pose.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, altitude));

    nav_msgs::Odometry gps_odom;

    // 좋은 헤딩이 아니라면
    if (!transform_good_)
    { 
      // Error msg 출력
      ROS_ERROR("No transform available (yet)");
      return false;
    }

    // 응답된 map point = 데카르트tomap 함수(데카르트 포즈를 변수로 받는)의 position
    response.map_point = cartesianToMap(cartesian_pose).pose.pose.position;

    return true;
  }

  // UTM 영역 서비스에 대한 콜백
  bool NavSatTransform::setUTMZoneCallback(robot_localization::SetUTMZone::Request& request,
                                           robot_localization::SetUTMZone::Response& response)
  {
    double x_unused;
    double y_unused;
    int prec_unused;

    // MGRS좌표를 UTM좌표로 반환하는 영역으로 영역, 북반구, 동쪽 x(미터), 북쪽 y(미터)로 변환
    GeographicLib::MGRS::Reverse(request.utm_zone, utm_zone_, northp_, x_unused, y_unused, prec_unused, true);
    ROS_INFO("UTM zone set to %d %s", utm_zone_, northp_ ? "north" : "south");
    return true;
  }

  //! 전달된 포즈를 utm에서 맵 프레임으로 변환
  //! input : cartesian_pose > 변환에 사용할 데카르트 좌표계의 포즈
  nav_msgs::Odometry NavSatTransform::cartesianToMap(const tf2::Transform& cartesian_pose) const
  {
    nav_msgs::Odometry gps_odom{};

    // 데카르트 gps로 변환
    tf2::Transform transformed_cartesian_gps{};

    // 변환된 데카르트 gps =  데카르트->odom 변환 유지 X 데카르트 포즈
    transformed_cartesian_gps.mult(cartesian_world_transform_, cartesian_pose);
    // 변환된 데카르트 gps는 쿼터니안 형식의 getIdentity를 Rotation으로 세팅
    transformed_cartesian_gps.setRotation(tf2::Quaternion::getIdentity());

    // Set header information stamp because we would like to know the robot's position at that timestamp
    // 해당 타임스탬프에서 로봇의 위치를 알고 싶기 때문에 헤더 정보 스탬프를 설정

    // GPS 주행 거리 측정 출력의 프레임 ID
    gps_odom.header.frame_id = world_frame_id_;
    // 최신 GPS 메시지의 간략한 타임스탬프
    gps_odom.header.stamp = gps_update_time_;

    // Now fill out the message. Set the orientation to the identity.
    // 이제 메시지를 작성하며 방향을 ID로 설정

    // toMsg : PoseStamped 메시지를 동등한 tf2 표현으로 변환
    tf2::toMsg(transformed_cartesian_gps, gps_odom.pose.pose);
    //gps_odom position z 설정 : 0 고도 보고 여부가 true면 0 아니면 gps_odom.pose.pose.position.z
    gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

    return gps_odom;
  }

  //! 전달된 지점을 지도 프레임에서 위도/경도로 변환
  //! input : point > 변환에 사용할 맵 프레임의 점을 가리킴
  void NavSatTransform::mapToLL(const tf2::Vector3& point, double& latitude, double& longitude, double& altitude) const
  { 
    // 데카르트로서의 odom 변환
    tf2::Transform odom_as_cartesian{};

    // 위치 tf
    tf2::Transform pose{};
    // pose의 point를 origin으로 설정
    pose.setOrigin(point);
    // pose의 쿼터니안 형식의 getidentity를 rotation으로 설정
    pose.setRotation(tf2::Quaternion::getIdentity());

    // odom as 데카르트 = 필터링된 GPS 브로드캐스트에 대한 odom->UTM x pose  
    odom_as_cartesian.mult(cartesian_world_trans_inverse_, pose);
    // odom as 데카르트는 쿼터니안 형식의 getidentity를 rotation으로 설정
    odom_as_cartesian.setRotation(tf2::Quaternion::getIdentity());

    // 로컬 데카르트(탄젠트 평면 ENU) 또는 UTM 좌표를 데카르트 좌표로 사용한다면
    if (use_local_cartesian_)
    {
      double altitude_tmp = 0.0;
      // 로컬 데카르트 x , y , z (미터)에서 측지 좌표 lat , lon (도), h (미터)로 변환
      // odom as 데카르트에서 x를 얻어 원점으로 ,y를 얻어 원점으로, 0, 위도, 경도, 고도를 얻음
      gps_local_cartesian_.Reverse(odom_as_cartesian.getOrigin().getX(),
                                   odom_as_cartesian.getOrigin().getY(),
                                   0.0,
                                   latitude,
                                   longitude,
                                   altitude_tmp);
      // 고도 = odom as 데카르트에서 z를 얻어 origin으로 함                             
      altitude = odom_as_cartesian.getOrigin().getZ();
    }
    // UTM 좌표를 데카르트 좌표로 사용하지 않는다면
    else
    { 
      // UTM 좌표를 지리적 좌표로 변환 utmzone, 북반구, x,y,위도 경도를 변수로
      GeographicLib::UTMUPS::Reverse(utm_zone_,
                                     northp_,
                                     odom_as_cartesian.getOrigin().getX(),
                                     odom_as_cartesian.getOrigin().getY(),
                                     latitude,
                                     longitude);
      // 고도 = odom as 데카르트에서 z를 얻어 origin으로 함                                 
      altitude = odom_as_cartesian.getOrigin().getZ();
    }
  }

  // 데카르트 좌표계에서 navsat 센서의 포즈가 주어지면 차량의 중심에서 오프셋을 제거하고 해당 중심의 '데카르트' 좌표 포즈를 반환
  void NavSatTransform::getRobotOriginCartesianPose(const tf2::Transform &gps_cartesian_pose,
                                                    tf2::Transform &robot_cartesian_pose,
                                                    const ros::Time &transform_time)
  { 
    // 행렬을 항등 행렬로 바꿈
    robot_cartesian_pose.setIdentity();

    // Get linear offset from origin for the GPS
    // GPS의 원점에서 선형 오프셋을 얻음
    tf2::Transform offset;

    //! 이 메서드는 특정 @p time 에 @p sourceFrame에서 @p targetFrame으로의 변환을 얻으려고 시도
    //! 그 때 사용할 수 있는 변환이 없으면 단순히 최신 변환을 얻으려고 시도
    //! 그래도 실패하면 메서드는 변환이 지정된 frame_id에서 그자체로 이동하는지 확인
    //! 하나라도 성공하면 메서드는 @p targetFrameTrans의 값을 설정하고 true를 반환, 그렇지 않으면 false를 반환
    /* bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
     *                   const std::string &targetFrame,
     *                    const std::string &sourceFrame,
     *                    const ros::Time &time,
     *                    const ros::Duration &timeout,
     *                    tf2::Transform &targetFrameTrans,
     */                    const bool silent = false);
    // 즉 gps frame id > base link frame id 변환 얻을려고 함
    // 실패시 tf_silent_failure_ 얻음
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 ros::Duration(transform_timeout_),
                                                                 offset,
                                                                 tf_silent_failure_);

    // 만약 변환이 된다면
    if (can_transform)
    {
      // Get the orientation we'll use for our Cartesian->world transform
      // Cartesian->world 변환에 사용할 방향을 가져옴
      
      // 쿼터니안 데카르트 오리엔테이션 = 변환 오리엔테이션
      tf2::Quaternion cartesian_orientation = transform_orientation_;
      // 데카르트 오리엔테이션은 3x3 행렬
      tf2::Matrix3x3 mat(cartesian_orientation);

      // Add the offsets
      // 오프셋 추가
      double roll;
      double pitch;
      double yaw;
      // 데카르트 오리엔테이션은 r,p,y를 RPY로 가짐
      mat.getRPY(roll, pitch, yaw);
      // yaw = yaw + 자기 편각 + yaw 오프셋 + utm 자오선 수렴
      yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
      // 데카르트 오리엔테이션은 r,p,y를 RPY로 세팅
      cartesian_orientation.setRPY(roll, pitch, yaw);

      // Rotate the GPS linear offset by the orientation
      // Zero out the orientation, because the GPS orientation is meaningless, and if it's non-zero, it will make the
      // the computation of robot_cartesian_pose erroneous.
      // GPS 방향이 무의미하기 때문에 방향을 0으로 하여 GPS 선형 오프셋을 회전
      // 방향이 0이 아니면 robot_cartesian_pose의 계산이 잘못됨

      // 오프셋의 origin을 (데카르트 오리엔테이션을 쿼터니안으로 변경) 세팅
      offset.setOrigin(tf2::quatRotate(cartesian_orientation, offset.getOrigin()));
      // 오프셋을 쿼터니안 형식의 getidentity의 rotation으로 세팅
      offset.setRotation(tf2::Quaternion::getIdentity());

      // Update the initial pose
      // 초기 포즈 업데이트

      // 로봇 데카르트 포즈 = 오프셋의 인벌스 * gps 데카르트 포즈 > gps위치+오프셋=로봇원점 당연한거임
      robot_cartesian_pose = offset.inverse() * gps_cartesian_pose;
    }
    // 변환 실패시
    else
    {
      // GPS 메시지의 frame_id(장착 위치 지정)이 ""가 아니라면
      if (gps_frame_id_ != "")
      { 
        // ROS Error msg 출력
        ROS_WARN_STREAM_ONCE("Unable to obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
          " transform. Will assume navsat device is mounted at robot's origin");
      }

      // 로봇 데카르트 포즈 = gps 데카르트 포즈
      robot_cartesian_pose = gps_cartesian_pose;
    }
  }

  //! 월드 프레임에서 navsat 센서의 포즈가 주어지면 차량의 중심에서 오프셋을 제거하고 해당 중심의 '월드' 프레임 포즈를 반환
  void NavSatTransform::getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                                tf2::Transform &robot_odom_pose,
                                                const ros::Time &transform_time)
  {
    // 행렬을 항등 행렬로 바꿈
    robot_odom_pose.setIdentity();

    // Remove the offset from base_link
    // base_link에서 오프셋 제거
    tf2::Transform gps_offset_rotated;

    //! 이 메서드는 특정 @p time 에 @p sourceFrame에서 @p targetFrame으로의 변환을 얻으려고 시도
    //! 그 때 사용할 수 있는 변환이 없으면 단순히 최신 변환을 얻으려고 시도
    //! 그래도 실패하면 메서드는 변환이 지정된 frame_id에서 그자체로 이동하는지 확인
    //! 하나라도 성공하면 메서드는 @p targetFrameTrans의 값을 설정하고 true를 반환, 그렇지 않으면 false를 반환
    /* bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
     *                   const std::string &targetFrame,
     *                    const std::string &sourceFrame,
     *                    const ros::Time &time,
     *                    const ros::Duration &timeout,
     *                    tf2::Transform &targetFrameTrans,
     */                    const bool silent = false);
    // 즉 gps frame id > base link frame id 변환 얻을려고 함
    // 실패시 tf_silent_failure_ 얻음
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 transform_timeout_,
                                                                 gps_offset_rotated,
                                                                 tf_silent_failure_);
    // 만약 tf한다면
    if (can_transform)
    {
      // 로봇 오리엔테이션 tf
      tf2::Transform robot_orientation;
      //! 이 메서드는 특정 @p time 에 @p sourceFrame에서 @p targetFrame으로의 변환을 얻으려고 시도
      //! 그 때 사용할 수 있는 변환이 없으면 단순히 최신 변환을 얻으려고 시도
      //! 그래도 실패하면 메서드는 변환이 지정된 frame_id에서 그자체로 이동하는지 확인
      //! 하나라도 성공하면 메서드는 @p targetFrameTrans의 값을 설정하고 true를 반환, 그렇지 않으면 false를 반환
      /* bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
      *                   const std::string &targetFrame,
      *                    const std::string &sourceFrame,
      *                    const ros::Time &time,
      *                    const ros::Duration &timeout,
      *                    tf2::Transform &targetFrameTrans,
      */                    const bool silent = false);
      // 즉 world_frame_id > base_link_frame_id_ 변환 얻을려고 함
      // 실패시 tf_silent_failure_ 얻음
      can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                              world_frame_id_,
                                                              base_link_frame_id_,
                                                              transform_time,
                                                              transform_timeout_,
                                                              robot_orientation,
                                                              tf_silent_failure_);
      // 만약 tf한다면
      if (can_transform)
      {
        // Zero out rotation because we don't care about the orientation of the
        // GPS receiver relative to base_link
        // base_link에 대한 GPS 수신기의 방향을 신경 쓰지 않기 때문에 회전을 0으로 만듬

        // (base_link에서 오프셋 제거한) 회전된 gps 오프셋의 origin은 로보 오리엔테이션의 회전을 얻은것과
        // 회전된 gps 오프셋에서 원점을 얻은거의 쿼터니안 회전
        gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
        // 회전된 gps 오프셋은 쿼터니안 형식의 getIdentity을 Rotation으로 세팅
        gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
        // 로봇 오돔 포즈 = 회전된 gps 오프셋의 인벌스 * gps 오돔 포즈(gps센서의 포즈)
        robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
      }
      // 만약 tf안한다면
      else
      {
        // Ros warning msg 출력
        ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << world_frame_id_ << "->" << base_link_frame_id_ <<
          " transform. Will not remove offset of navsat device from robot's origin.");
      } // World frame > base frame
    }
    // 만약 tf안한다면
    else
    {
      // Ros warning msg 출력
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
        " transform. Will not remove offset of navsat device from robot's origin.");
    }  // base frame > gps frame
  }

  //! GPS 수정 데이터에 대한 콜백
  //! input : msg > 처리할 NavSatFix 메시지
  void NavSatTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  { 
    // gps frame id = 헤더 frame id를 출력하라
    gps_frame_id_ = msg->header.frame_id;

    // gps frame id가 비었을 때
    if (gps_frame_id_.empty())
    { 
      // 오류 출력
      ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
        "origin.");
    }

    // Make sure the GPS data is usable
    // GPS 데이터를 사용할 수 있는지 확인
    // msg > 상태가 STATUS_NO_FIX와 다르고 위도, 경도, 고도가 nan이 아니라면
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                     !std::isnan(msg->altitude) &&
                     !std::isnan(msg->latitude) &&
                     !std::isnan(msg->longitude));

    // 만약 GPS 데이터를 사용할 수 있다면
    if (good_gps)
    {
      // If we haven't computed the transform yet, then
      // store this message as the initial GPS data to use
      // 아직 변환을 계산하지 않은 경우 이 메시지를 사용할 초기 GPS 데이터로 저장
      // 나쁜 헤딩을 했고 첫 번째 GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 안가져왔으면
      if (!transform_good_ && !use_manual_datum_)
      { 
        // 변환에 사용할 NavSatFix 메시지를 input으로 넣어 GPS 데이터를 설정
        setTransformGps(msg);
      }

      double cartesian_x = 0.0;
      double cartesian_y = 0.0;
      double cartesian_z = 0.0;
      
      // UTM 좌표를 데카르트 좌표로 사용한다면
      if (use_local_cartesian_)
      { 
        // GPS 원점 주변의 로컬 데카르트 투영법
        gps_local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude,
                                     cartesian_x, cartesian_y, cartesian_z);
      }
      // UTM 좌표를 데카르트 좌표로 사용하지 않으면
      else
      {
        // Transform to UTM using the fixed utm_zone_
        // 고정 utm_zone_을 사용하여 UTM으로 변환
        int zone_tmp;
        bool northp_tmp;
        //예외 발생에 대한 검사의 범위를 지정 (예외 배제) : GeographicLib::UTMUPS를 해라
        try
        {
          GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude,
                                        zone_tmp, northp_tmp, cartesian_x, cartesian_y, utm_zone_);
        }
        // 예외 상황이 생김
        catch (const GeographicLib::GeographicErr& e)
        {
          // printf 스타일 형식을 사용하여 특정 인쇄 속도로 제한된 지정된 상세 수준에서 지정된 명명된 로거에 기록
          // what() 은 const char을 반환하는 가상 멤버 함수 즉, 위의 const GeographicLib::GeographicErr& e 반환
          ROS_ERROR_STREAM_THROTTLE(1.0, e.what());
          return;
        }
      }
      // 데카르트 좌표로 저장된 최신 GPS 데이터의 origin은 (벡터형 데카르트 x,y,고도)로 set
      latest_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
      // 최신 데카르트 좌표로 저장된 최신 GPS 데이터의 공분산은 zero로 set
      latest_cartesian_covariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      // 나중에 회전할 수 있도록 측정의 공분산 행렬을 복사
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          latest_cartesian_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
        }
      }

      gps_update_time_ = msg->header.stamp;
      gps_updated_ = true;
    }
  }

  //! IMU 데이터에 대한 콜백
  //! input : msg > 처리할 IMU 메시지
  void NavSatTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // We need the baseLinkFrameId_ from the odometry message, so
    // we need to wait until we receive it.
    // odom 메시지에서 baseLinkFrameId_가 필요하므로 수신할 때까지 기다려야 함
    if (has_transform_odom_)
    {
      /* This method only gets called if we don't yet have the
       * IMU data (the subscriber gets shut down once we compute
       * the transform), so we can assumed that every IMU message
       * that comes here is meant to be used for that purpose. */
      // 이 메소드는 아직 IMU 데이터가 없는 경우에만 호출되므로(변환을 계산하면 구독자가 종료됨)
      // 여기에 오는 모든 IMU 메시지가 해당 목적으로 사용된다고 가정
      // fromMsg(A,B) A:ROSmsg B:변환할 객체 : Ros type A를 Another type B로 변환
      tf2::fromMsg(msg->orientation, transform_orientation_);

      // Correct for the IMU's orientation w.r.t. base_link
      // IMU의 오리엔테이션 w,r,t base_link의 보정
      tf2::Transform target_frame_trans;
      //! 이 메서드는 특정 @p time 에 @p sourceFrame에서 @p targetFrame으로의 변환을 얻으려고 시도
      //! 그 때 사용할 수 있는 변환이 없으면 단순히 최신 변환을 얻으려고 시도
      //! 그래도 실패하면 메서드는 변환이 지정된 frame_id에서 그자체로 이동하는지 확인
      //! 하나라도 성공하면 메서드는 @p targetFrameTrans의 값을 설정하고 true를 반환, 그렇지 않으면 false를 반환
      /* bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
       *                    const std::string &targetFrame,
       *                    const std::string &sourceFrame,
       *                    const ros::Time &time,
       *                    const ros::Duration &timeout,
       *                    tf2::Transform &targetFrameTrans,
       */                   const bool silent = false);
      // 즉 header.frame_id > base_link_frame_id_ 변환 얻을려고 함
      // 실패시 tf_silent_failure_ 얻음
      bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                   base_link_frame_id_,
                                                                   msg->header.frame_id,
                                                                   msg->header.stamp,
                                                                   transform_timeout_,
                                                                   target_frame_trans,
                                                                   tf_silent_failure_);

      // tf할 수 있으면
      if (can_transform)
      { 
        double roll_offset = 0;
        double pitch_offset = 0;
        double yaw_offset = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        //! input : quat > 변환할 쿼터니언
        //! output : roll > 변환된 roll
        //! output : pitch > 변환된 pitch
        //! output : yaw > 변환된 yaw
        //! 👉️ 함수 : 쿼터니안, r, p, y를 멤버변수로 받는 quatToRPY 멤버 함수

        // 변환된 r,p,y_offset 멤버변수를 받아 target_frame_trans의 rotation을 얻음
        RosFilterUtilities::quatToRPY(target_frame_trans.getRotation(), roll_offset, pitch_offset, yaw_offset);
        // r,p,y 멤버 변수로 받아 transform_orientation(최신 imu오리엔테이션)을 변환하여 쿼터니언으로 얻음
        RosFilterUtilities::quatToRPY(transform_orientation_, roll, pitch, yaw);

        // Debug msg 출력
        ROS_DEBUG_STREAM("Initial orientation is " << transform_orientation_);

        // Apply the offset (making sure to bound them), and throw them in a vector
        // 오프셋을 적용(바운딩 확인)하고 벡터에 던짐
        // vector형 rpy_angle은 로테이션 범위를 정해줌 (r-r.offset ...)
        tf2::Vector3 rpy_angles(FilterUtilities::clampRotation(roll - roll_offset),
                                FilterUtilities::clampRotation(pitch - pitch_offset),
                                FilterUtilities::clampRotation(yaw - yaw_offset));

        // Now we need to rotate the roll and pitch by the yaw offset value.
        // Imagine a case where an IMU is mounted facing sideways. In that case
        // pitch for the IMU's world frame is roll for the robot.
        // 이제 요 오프셋 값으로 롤과 피치를 회전해야 함
        // IMU가 옆을 향하여 장착되는 경우를 상상
        // 이 경우 IMU의 세계 프레임에 대한 피치는 로봇에 대한 롤 임

        // 행렬 3x3 mat정의
        tf2::Matrix3x3 mat;
        // 행렬 mat은 0 0 yaw.offset으로 RPY 세팅
        mat.setRPY(0.0, 0.0, yaw_offset);
        // rpy_angle = (0 0 yaw.offset) x (rpy_angles)
        rpy_angles = mat * rpy_angles;
        // imu의 오리엔테이션 값은 rpy_angle의 x,y,z로 RPY를 세팅
        transform_orientation_.setRPY(rpy_angles.getX(), rpy_angles.getY(), rpy_angles.getZ());

        // DEBUG Error msg 출력
        ROS_DEBUG_STREAM("Initial corrected orientation roll, pitch, yaw is (" <<
                         rpy_angles.getX() << ", " << rpy_angles.getY() << ", " << rpy_angles.getZ() << ")");

        has_transform_imu_ = true;
      }
    }
  }

  //! odom 데이터에 대한 콜백
  //! input : msg > 처리할 odom 메시지
  void NavSatTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  { 
    // world frame id가 header frame id
    // base link frame id가 child frame id
    world_frame_id_ = msg->header.frame_id;
    base_link_frame_id_ = msg->child_frame_id;

    // 헤딩 좋지않고, GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터를 안가져왔으면
    if (!transform_good_ && !use_manual_datum_)
    { 
      // 변환에 사용될 msg를 갖고 odom 데이터를 설정
      setTransformOdometry(msg);
    }

    // fromMsg(A,B) A:ROSmsg B:변환할 객체 : Ros type A를 Another type B로 변환
    tf2::fromMsg(msg->pose.pose, latest_world_pose_);
    // 최근 오돔 공분산은 zero로 set
    latest_odom_covariance_.setZero();
    // 0 < 행 < Pose Size
    for (size_t row = 0; row < POSE_SIZE; ++row)
    { 
      // 0 < 열 < Pose Size
      for (size_t col = 0; col < POSE_SIZE; ++col)
      { 
        // 최신 오돔 공분산
        latest_odom_covariance_(row, col) = msg->pose.covariance[row * POSE_SIZE + col];
      }
    }

    odom_update_time_ = msg->header.stamp;
    odom_updated_ = true;
  }

  //! odom 데이터를 다시 GPS로 변환하고 브로드캐스트
  //! output : filtered_gps > 준비할 NavSatFix 메시지
  bool NavSatTransform::prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps)
  {
    bool new_data = false;

    // 좋은 헤딩을 계산했고 새로운 odom 데이터가 있으면
    if (transform_good_ && odom_updated_)
    { 
      // 전달된 지점을 지도 프레임에서 위도/경도로 변환
      // 여기서 변환될 map point는 atest_world_pose_.getOrigin()임
      mapToLL(latest_world_pose_.getOrigin(), filtered_gps.latitude, filtered_gps.longitude, filtered_gps.altitude);

      // Rotate the covariance as well
      // 공분산도 회전

      // 포즈의 회전 구성 요소(필터링된 GPS 브로드캐스트에 대한 odom->UTM 변환을 유지를 rotation으로 얻음)
      tf2::Matrix3x3 rot(cartesian_world_trans_inverse_.getRotation());
      // 행렬 rot_6d(pose size, pose size)
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      // 행렬을 항등 행렬로 바꿈
      // 항등행렬(Identity Matrices)이란 모든 대각원소들이 1 이며 나머지 원소들이 0인 정방행렬
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        // const int POSITION_SIZE = 3;
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      // 공분산 회전
      // 최신 오돔 공분산 = 항등행렬 rot_6d * 최신 odom 공분산(문자로 표현된 코드 실행) * rot_6d의 전치행렬
      // 전치행렬 : 행렬의 행과 열을 바꾸기
      latest_odom_covariance_ = rot_6d * latest_odom_covariance_.eval() * rot_6d.transpose();

      // Copy the measurement's covariance matrix back
      // 측정의 공분산 행렬을 다시 복사
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          filtered_gps.position_covariance[POSITION_SIZE * i + j] = latest_odom_covariance_(i, j);
        }
      }

      filtered_gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      filtered_gps.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      filtered_gps.header.frame_id = base_link_frame_id_;
      filtered_gps.header.stamp = odom_update_time_;

      // Mark this GPS as used
      odom_updated_ = false;
      new_data = true;
    }

    return new_data;
  }

  //! 전송하기 전에 GPS 주행 거리 측정 메시지를 준비
  //! output : gps_odom > gps_odom 준비할 주행 거리 측정 메시지
  bool NavSatTransform::prepareGpsOdometry(nav_msgs::Odometry &gps_odom)
  {
    bool new_data = false;
    
    // 좋은 헤딩을 계산했고 새로운 gps 데이터가 있고 새로운 odom 데이터가 있으면
    if (transform_good_ && gps_updated_ && odom_updated_)
    { 
      //gps odom = 변환에 사용할 데카르트 좌표계의 포즈(데카르트 좌표로 저장된 최신 GPS 데이터)를 utm에서 맵 프레임으로 변환
      gps_odom = cartesianToMap(latest_cartesian_pose_);

      // 변환된 데카르트 gps tf
      tf2::Transform transformed_cartesian_gps;
      // ROS TYPE gps_odom.pose.pose을 변환된 데카르트 gps type으로 변경
      tf2::fromMsg(gps_odom.pose.pose, transformed_cartesian_gps);

      // Want the pose of the vehicle origin, not the GPS
      // GPS가 아닌 차량 원점의 포즈를 원함

      // 변환된 데카르트 로봇 tf
      tf2::Transform transformed_cartesian_robot;
      // 월드 프레임에서 navsat 센서의 포즈가 주어지면 차량의 중심에서 오프셋을 제거하고 해당 중심의 월드 프레임 포즈를 반환
      getRobotOriginWorldPose(transformed_cartesian_gps, transformed_cartesian_robot, gps_odom.header.stamp);

      // Rotate the covariance as well
      // 공분산도 회전

      // 포즈의 회전 구성 요소(데카르트->odom 변환 유지를 rotation으로 얻음)
      tf2::Matrix3x3 rot(cartesian_world_transform_.getRotation());
      // 행렬 rot_6d(pose size, pose size)
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      // 행렬을 항등 행렬로 바꿈
      // 항등행렬(Identity Matrices)이란 모든 대각원소들이 1 이며 나머지 원소들이 0인 정방행렬
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      // 공분산 회전

      // 최신 데카르트 공분산 = 항등행렬 rot_6d * 최신 데카르트 공분산(문자로 표현된 코드 실행) * rot_6d의 전치행렬
      latest_cartesian_covariance_ = rot_6d * latest_cartesian_covariance_.eval() * rot_6d.transpose();

      // Now fill out the message. Set the orientation to the identity.
      // 이제 메시지를 작성해라 방향을 ID로 설정
      tf2::toMsg(transformed_cartesian_robot, gps_odom.pose.pose);
      gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

      // Copy the measurement's covariance matrix so that we can rotate it later
      // ❓️ 나중에 회전할 수 있도록 측정의 공분산 행렬을 복사 ❓️
      for (size_t i = 0; i < POSE_SIZE; i++)
      {
        for (size_t j = 0; j < POSE_SIZE; j++)
        {
          gps_odom.pose.covariance[POSE_SIZE * i + j] = latest_cartesian_covariance_(i, j);
        }
      }

      // Mark this GPS as used
      gps_updated_ = false;
      new_data = true;
    }

    return new_data;
  }

  //! 변환을 계산하는 데 사용할 GPS 데이터를 설정하는 데 사용
  //! input : msg > 변환에 사용할 NavSatFix 메시지
  void NavSatTransform::setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    double cartesian_x = 0;
    double cartesian_y = 0;
    double cartesian_z = 0;

    // UTM 좌표를 데카르트 좌표로 사용한다면
    if (use_local_cartesian_)
    { 
      const double hae_altitude = 0.0;
      // 리셋 후 다시 재정의
      gps_local_cartesian_.Reset(msg->latitude, msg->longitude, hae_altitude);
      gps_local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, cartesian_x, cartesian_y, cartesian_z);

      // UTM meridian convergence is not meaningful when using local cartesian, so set it to 0.0
      // UTM 자오선 수렴은 지역 데카르트를 사용할 때 의미가 없으므로 0.0으로 설정
      utm_meridian_convergence_ = 0.0;
    }
    // UTM 좌표를 데카르트 좌표로 사용하지 않는다면
    else
    {
      double k_tmp;
      double utm_meridian_convergence_degrees; // utm의 자오선 수렴 각도
      GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone_, northp_,
                                     cartesian_x, cartesian_y, utm_meridian_convergence_degrees, k_tmp);
      // utm 자오선 수렴 = utm 자오선 수렴 각도 * 각도 당 라디안                   
      utm_meridian_convergence_ = utm_meridian_convergence_degrees * NavsatConversions::RADIANS_PER_DEGREE;
    }

    // ROS msg 출력
    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " <<
                    msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum " << ((use_local_cartesian_)? "Local Cartesian" : "UTM") <<
                    " coordinate is (" << std::fixed << cartesian_x << ", " << cartesian_y << ") zone " << utm_zone_);

    // gps odom pose의 origin은 vector 형식의 (데카르트 x,y,고도)
    transform_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
    // gps odom pose는 쿼터니안 형식의 항등행렬을 rotation으로 세팅
    transform_cartesian_pose_.setRotation(tf2::Quaternion::getIdentity());
    has_transform_gps_ = true;
  }

  //! 변환을 계산하는 데 사용할 odom 데이터를 설정하는 데 사용
  //! input : msg > 변환에 사용할 odom 메시지
  void NavSatTransform::setTransformOdometry(const nav_msgs::OdometryConstPtr& msg)
  { 
    // pose.pose를 transform world pose type(최신 IMU 오리엔테이션)으로 변경
    tf2::fromMsg(msg->pose.pose, transform_world_pose_);
    has_transform_odom_ = true;

    // Ros msg 출력
    ROS_INFO_STREAM_ONCE("Initial odometry pose is " << transform_world_pose_);

    // Users can optionally use the (potentially fused) heading from
    // the odometry source, which may have multiple fused sources of
    // heading data, and so would act as a better heading for the
    // Cartesian->world_frame transform.
    // 사용자는 선택적으로 odom 소스에서 (잠재적으로 융합된) 헤딩(방향)을 사용할 수 있음
    // 이 헤딩는 여러 통합된 헤딩 데이터 소스를 가질 수 있으므로 Cartesian->world_frame 변환에 대한 더 나은 헤딩 역할을 함

    // 좋은 헤딩 계산 안했고, 주행 거리 또는 IMU 소스에서 변환의 yaw을 가져오고, GPS 메시지 또는 set_datum 서비스/매개변수에서 데이터 안가져오면
    if (!transform_good_ && use_odometry_yaw_ && !use_manual_datum_)
    {
      sensor_msgs::Imu *imu = new sensor_msgs::Imu();
      imu->orientation = msg->pose.pose.orientation;
      imu->header.frame_id = msg->child_frame_id;
      imu->header.stamp = msg->header.stamp;
      sensor_msgs::ImuConstPtr imuPtr(imu);
      imuCallback(imuPtr);
    }
  }

}  // namespace RobotLocalization
