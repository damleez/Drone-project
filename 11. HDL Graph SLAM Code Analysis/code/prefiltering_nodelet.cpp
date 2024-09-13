// SPDX-License-Identifier: BSD-2-Clause

#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nordelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace hdl_graph_slam {

class PrefilteringNodelet : public nodelet::Nodelet { //PrefilteringNodelet이 nodelet class가 됨
public:
  typedef pcl::PointXYZI PointT; //point cloude를 x,y,z,intensity를 받아서 이름을 PointT로 둠

  PrefilteringNodelet() {} //생성자
  virtual ~PrefilteringNodelet() {} //가상소멸자 > 안쓰면 부모클래스 소멸자만 삭제됨, 자식 클래스의 소멸자는 결코 호출 X


  virtual void onInit() { //가상함수 OnInit (초기화) : 가상함수는 public 섹션에 선언
    nh = getNodeHandle(); //node handle 생성 > nodehandle class에 advertise존재(sub하고자 하는 노드에게 그 토픽으로 pub하고자 하는 노드가 있음을 전달)
    private_nh = getPrivateNodeHandle();

    initialize_params(); //⭐가상함수의 멤버함수 맞나?⭐

    if(private_nh.param<bool>("deskewing", false)) { //imu는 optional sensor로 true시 imu_sub / false시 imu sub X
      imu_sub = nh.subscribe("/imu/data", 1, &PrefilteringNodelet::imu_callback, this);
    }

    //nodehandle로 세개의 주제를 pub or sub
    points_sub = nh.subscribe("/velodyne_points", 64, &PrefilteringNodelet::cloud_callback, this);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
    colored_pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_points", 32);
  }
⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘 모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
initialize_params < 이거 public에서 초기화 시켰고, private에서는 함수로 받음.. 왜 굳이? 모슨의미고
: 일반적으로 멤버 변수는 비공개로 하고, 멤버 함수는 공개하는 것이 일반적이라고 함
⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

private: //Downsampling filtering method
  //private해서 직접 접근할 수 없지만 public 멤버 함수인 initialize_params()를 통해 간접적으로 접근 가능.. 맞나 ?

  ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘 모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
  pcl 사용시 멤버 변수에서 사용해야할 때는 선언을 따로 해주고, 생성자(costructor)에서 동적할당(new)을 해서 써야함
  근데 아래는 생성자아닌데,, 생성자는 반환형 절대 X기 때문
  근데 왜 여기서 동적할당을 해줬을까?
  ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

  void initialize_params() {
    std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    //1. VOXELGRID일 때,
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      //auto 키워드는 초기화시에 초기화 값에 맞춰 자동으로 자료형을 판단하는 기능
      //pointXYZI를 PointT로 선언했으며, 그 PointT크기의 자료형을 voxelgrid 포인터에 주소를 저장
      //동적할당 ? 크기가 정확히 얼마나 되는지 모르는 경우, 메모리가 낭비되거나 과부화되는 것을 막을려고 동적 할당해줌 
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      //voxelgrid의 쪼개는 크기는 downsample_resolution크기로 자름 (위에서 0.1로 정의 = 10cm)
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      //downsample_filter를 voxelgrid로 reset
      downsample_filter.reset(voxelgrid);
    //2. APPROX_VOXELGRID일 때,
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      //pcl::ApproximateVoxelGrid의 Ptr은 아래와 같이 선언해서 사용 > approx_voxelgrid로 받겠다는 뜻
      //new를 사용해서 요청한 타입의 객체를 담을 수 있는 크기의 메모리를 할당해 주어야 함
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      //approximatevoxelgrid의 쪼개는 크기는 downsample_resolution크기로 자름 (위에서 0.1로 정의 = 10cm)
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      //downsample filter는 Approximate Voxel Grid이다
      downsample_filter = approx_voxelgrid;
    //3. 다운 샘플링이 없을 때, error message 출력
    } else {
      //downsample method가 없으면
      if(downsample_method != "NONE") {
        //error msg 출력
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    //Outlier Removal Method
    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    //1. 만약 outlier removal method가 Statistical를 기준으로 정의 된다면
    if(outlier_removal_method == "STATISTICAL") {
      //이웃 수 20으로 설정
      int mean_k = private_nh.param<int>("statistical_mean_k", 20);
      //쿼리 포인트(기준이 되는 포인트)까지의 평균 거리의 1 표준편차보다 큰 거리를 갖는 모든 포인트를 정의 
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      //StatisticalOutlierRemoval의 포인터 sor은 new로 메모리 할당해주면서 선언됨
      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      //MeanK를 내가 설정한 20개의 이웃을 갖는 mean_k로 set
      sor->setMeanK(mean_k);
      //내가 설정해준 표준편차 1로 set
      sor->setStddevMulThresh(stddev_mul_thresh);
      //outlier_removal_filter는 sor과 같음 > 이웃 20, 평균 거리 표준표차 1
      outlier_removal_filter = sor;
    //2. 만약 outlier removal method가 Radius를 기준으로 정의 된다면
    } else if(outlier_removal_method == "RADIUS") {
      //반경 0.8로 정의
      double radius = private_nh.param<double>("radius_radius", 0.8);
      //최소 이웃수 2로 정의
      int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      //RadiusOutlierRemoval 포인터 rad은 new로 메모리 할당해주면서 선언됨
      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      //내가 설정해준 반경으로 set
      rad->setRadiusSearch(radius);
      //내가 설정해준 최소 이웃수로 set
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    //distance filter 사용은 bool형으로 true시 출력
    use_distance_filter = private_nh.param<bool>("use_distance_filter", true);
    //distance near thresh는 1로 설정
    distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
    //distance far thresh는 100로 설정
    distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0);
    //base link frame은 string형식으로 base_link_frame이라고 param설정
    base_link_frame = private_nh.param<std::string>("base_link_frame", "");
  }

  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    imu_queue.push_back(imu_msg);
  }

  void cloud_callback(const pcl::PointCloud<PointT>& src_cloud_r) {
    //makeshared는 shared_ptr에서 동적 할당 2번할 것을 1번으로 줄여줌
    //그리고 shared ptr은 같은 객체를 가리킬 수 있음(여러개)
    //여기서 ConstPtr은 상수 메시지에 대한 공유 포인터, Ptr은 (수정가능한) 메시지에 대한 공유 포인터
    pcl::PointCloud<PointT>::ConstPtr src_cloud = src_cloud_r.makeShared();
    if(src_cloud->empty()) {
      return;
    }

    src_cloud = deskewing(src_cloud);

    // if base_link_frame is defined, transform the input cloud to the frame
    // base_link_frame이 정의된 경우 입력 클라우드를 프레임으로 변환
    if(!base_link_frame.empty()) {
      //base link frame이 없으면, transform할 수 없으면 error msg 출력
      if(!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) {
        std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
      }
      
      tf::StampedTransform transform;
      //지정된 변환이 사용 가능할 때 까지 wait
      //TF 변환시 시간은 rospy.Time.now()가 아니라 Time(0)을 써야함
      tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
      //각각의 tf 리스너들은 서로 다른 tf 브로드캐스터들로 부터 오는 좌표 transform을 임시로 넣어둘 버퍼를 가지고 있음
      //tf 브로드캐스터가 transform을 전송할 때, 그 transform이 버퍼에 들어갈 때까지 보통 수ms 정도의 시간이 소요
      //그래서, waitfortransform 2초를 한 후, lookuptransform을 함
      tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);
      
      //PointCloud 포인터 transformed은 new로 메모리 할당해주면서 선언됨
      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      //src_cloud input : 입력 포인트 클라우드 / transformed : 입력 포인트 클라우드에서 사용할 포인트 인덱스세트 / transform : 아핀 변환(rigid)
      //src cloud를 transformed에 rigid 변환된 결과를 넣음
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
      transformed->header.frame_id = base_link_frame;
      transformed->header.stamp = src_cloud->header.stamp;
      src_cloud = transformed;
    }

    //filtered = distance filter 함수의 입력을 src_cloud로 받음
    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    //filtered는 downsample 및 outlier removal을 거친 결과값
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    //이러한 filtered를 최종적으로 pub해줌
    points_pub.publish(*filtered);
  }

  //downsample 함수
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    //downsample filter가 없다면 cloud만 return
    if(!downsample_filter) {
      return cloud;
    }

    //Point cloud 포인터 filtered를 new를 통해 할당
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    //downsample filter는 cloud를 inputcloud로 set
    downsample_filter->setInputCloud(cloud);
    //downsample filter는 filtered를 filter로 set
    downsample_filter->filter(*filtered);

    //filtered의 header를 cloud의 header와 같다고 하는 것인감? 그런듯 ㅇㅇ
    //std_msgs/Header은 unit32 시퀀스, 타임스탬프, 문자열등이 포함..ㅇㅇ
    filtered->header = cloud->header;

    return filtered;
  }

  //outlier removal 함수
  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    //outlier_removal_filter가 없다면 cloud만 return
    if(!outlier_removal_filter) {
      return cloud;
    }

    //Pointcloud의 포인터 filtered는 new로 할당
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  //distance filter 방법
  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    //reserve함수란 cloud의 사이즈가 기존 capacity보다 크면 메모리 재할당과 원소복사를 한 새로운 array를 만듦
    //하지만, n이 기존 capacity보다 작다면 아무런 일이 일어나지 않음
    filtered->reserve(cloud->size());

    //copy if는 처음부터 끝까지 조건을 만족하는 element를 copy
    //back inserter는 filtered->points해당하는 요소들을 먼저 적고 이후 cloud 요소들 마지막에 넣음
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      //d는 point p의 참조자 걍 pontT=t임 
      //p의 3차원 벡터 맵을 얻어 벡터의 크기/길이를 측정 (norm)
      double d = p.getVector3fMap().norm();
      //distance near thresh < d < distance far thresh 를 true or false로 return
      return d > distance_near_thresh && d < distance_far_thresh;
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  //deskewing 함수
  pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    //pcl 변환 : pcl의 cloud>header stamp를 ros의 타임 스탬프로 변환
    ros::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);
    //imu queue비어있으면 cloud return
    if(imu_queue.empty()) {
      return cloud;
    }

    // the color encodes the point number in the point sequence
    // 색상은 포인트 시퀀스의 포인트 번호를 인코딩함
    if(colored_pub.getNumSubscribers()) {
      //pointcloud XYZRGB뽑을 때, 포인터를 colored라고 하며 new로 할당
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
      colored->header = cloud->header;
      colored->is_dense = cloud->is_dense;
      colored->width = cloud->width;
      colored->height = cloud->height;
      colored->resize(cloud->size());

      for(int i = 0; i < cloud->size(); i++) {
        double t = static_cast<double>(i) / cloud->size();
        //at함수는 str.at(index)로 index위치의 문자 반환, 유효한 범위인지 체크 O
        //4f는 4x1 float형 matrix
        colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
        colored->at(i).r = 255 * t;
        colored->at(i).g = 128;
        colored->at(i).b = 255 * (1 - t);
      }
      colored_pub.publish(*colored);
    }

    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘 모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
    //imu_msg = imu queue의 가장 front의 element
    //그리고 loc이 무슨 의미인지도 모르겠음
    sensor_msgs::ImuConstPtr imu_msg = imu_queue.front();

    //begin은 iterator이므로 auto로 인해 loc이 interator형이 됨 vector<int>::iterator loc = imu_queue.begin() 요로케
    auto loc = imu_queue.begin();
    //for(초기화;조건;단계) 인데 초기화는 위의 식과 같음
    //iterator 때문에 계속해서 처음부터 끝까지 반복하는 반복문이됨, 포인터와 비슷한 객체
    //iterator의 장점은 index관리라고 함
    //컨테이너의 end는 끝이 아니라 맨 마지막 원소의 다음번 원소임, 만약 아무 원소도 없다면 begin==end
    for(; loc != imu_queue.end(); loc++) {
      //포인터처럼 역참조 연산으로 실제 값을 참조해야함
      //왜냐면 vector의 itertor가 포인터 그 잡채, 배열의 요소를 가르킴
      // imu iterator도는데 첫번째에서 imu_queue.front가 loc, 두번째에서 두번째 벡터 배열 imu_queue.front가 loc ... 반복
      imu_msg = (*loc);
      if((*loc)->header.stamp > stamp) {
        break;
      }
    }

    //imu queue 시작에서 loc(imu_queue의 front의 첫번째 element)을 지워라
    //근데... 왜지움?
    imu_queue.erase(imu_queue.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    //ang_v=ang_v * -1
    //여기서 -1은 왜 한건가 뭐 반대로 있겠지?
    ang_v *= -1;
    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

    ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐잘 모르겠음⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    //scan period를 0.1로 지정
    double scan_period = private_nh.param<double>("scan_period", 0.1);
    for(int i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      // IMU 데이터를 LIDAR 프레임으로 변환
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();
      //여기서 왜 2를 나누고 각속도를 곱하는건가 공식인가?
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }
  ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber imu_sub;
  std::vector<sensor_msgs::ImuConstPtr> imu_queue;

  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  ros::Publisher colored_pub;

  tf::TransformListener tf_listener;

  std::string base_link_frame;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PrefilteringNodelet, nodelet::Nodelet)
