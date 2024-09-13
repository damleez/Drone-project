#ifndef ROBOT_LOCALIZATION_NAVSAT_CONVERSIONS_H
#define ROBOT_LOCALIZATION_NAVSAT_CONVERSIONS_H

/**  @file

     @brief Universal Transverse Mercator transforms.
     Functions to convert (spherical) latitude and longitude to and
     from (Euclidean) UTM coordinates.
     Universal Transverse Mercator 변환
    🌟️ (구형) 위도와 경도를 (유클리드) UTM 좌표로 변환하는 기능 🌟️
 */

#include <cmath>    //수학 헤더 파일
#include <string>   //문자열 헤더 파일

#include <stdio.h>
#include <stdlib.h>

#include <GeographicLib/MGRS.hpp>   //군사용 글로벌 좌표계
#include <GeographicLib/UTMUPS.hpp> //UTM:투영방법을 사용한 2차원 좌표계, UPS:지역 좌표계

namespace RobotLocalization
{
namespace NavsatConversions
{
//라디안과 각도 변환
const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

// Grid granularity for rounding UTM coordinates to generate MapXY.
// MapXY를 생성하기 위해 UTM 좌표를 반올림하기 위한 그리드 세분성으로 100km 기준(원래 기본 그리드선 100km)
const double grid_size = 100000.0;    // 100 km grid

//! @brief gps_common reference
// WGS84 Parameters
// 지리좌표계 (3차원 - 위도,경도,높이)파라미터
#define WGS84_A   6378137.0   // major axis
#define WGS84_B   6356752.31424518  // minor axis
#define WGS84_F   0.0033528107    // ellipsoid flattening - 타원체 평평하게 하기
#define WGS84_E   0.0818191908    // first eccentricity - 이심률
#define WGS84_EP  0.0820944379    // second eccentricity

//! @brief gps_common reference
// UTM Parameters
#define UTM_K0    0.9996               // scale factor
#define UTM_FE    500000.0             // false easting
#define UTM_FN_N  0.0                  // false northing, northern hemisphere
#define UTM_FN_S  10000000.0           // false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E)    // e^2
#define UTM_E4    (UTM_E2*UTM_E2)      // e^4
#define UTM_E6    (UTM_E4*UTM_E2)      // e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2))  // e'^2

/**
 * Utility function to convert geodetic to UTM position
 * 측지 위치를 UTM 위치로 변환하는 유틸리티 기능 (3->2차원)
 * Units in are floating point degrees (sign for east/west)
 * 단위는 부동 소수점 도(동/서 기호)
 * Units out are meters
 * 단위는 미터
 * @todo deprecate this interface in favor of LLtoUTM()
 * LLtoUTM()을 위해 이 인터페이스를 더 이상 사용하지 않음
 */ 
static inline void UTM(double lat, double lon, double *x, double *y)
{
  // constants : 상수
  static const double m0 = (1 - UTM_E2/4 - 3*UTM_E4/64 - 5*UTM_E6/256);
  static const double m1 = -(3*UTM_E2/8 + 3*UTM_E4/32 + 45*UTM_E6/1024);
  static const double m2 = (15*UTM_E4/256 + 45*UTM_E6/1024);
  static const double m3 = -(35*UTM_E6/3072);

  // compute the central meridian
  // 본초 자오선을 계산 (삼항식)
  // UTM Zone은 경도 6도 간격
  int cm = ((lon >= 0.0)
    ? (static_cast<int>(lon) - (static_cast<int>(lon)) % 6 + 3)
    : (static_cast<int>(lon) - (static_cast<int>(lon)) % 6 - 3));

  // convert degrees into radians
  // 도를 라디안으로 변환
  double rlat = lat * RADIANS_PER_DEGREE;
  double rlon = lon * RADIANS_PER_DEGREE;
  double rlon0 = cm * RADIANS_PER_DEGREE;

  // compute trigonometric functions
  // 삼각 함수 계산
  double slat = sin(rlat);
  double clat = cos(rlat);
  double tlat = tan(rlat);

  // decide the false northing at origin
  // 원점에서 false northing을 결정
  // FN_N(북반구)이 0, FN_S(남반구)이 10000000.0 
  double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

  double T = tlat * tlat;
  double C = UTM_EP2 * clat * clat;
  double A = (rlon - rlon0) * clat;
  double M = WGS84_A * (m0*rlat + m1*sin(2*rlat)
      + m2*sin(4*rlat) + m3*sin(6*rlat));
  double V = WGS84_A / sqrt(1 - UTM_E2*slat*slat);  //Z

  // compute the easting-northing coordinates
  // easting-northing 좌표 계산
  *x = UTM_FE + UTM_K0 * V * (A + (1-T+C)*pow(A, 3)/6
            + (5-18*T+T*T+72*C-58*UTM_EP2)*pow(A, 5)/120);
  *y = fn + UTM_K0 * (M + V * tlat * (A*A/2
              + (5-T+9*C+4*C*C)*pow(A, 4)/24
              + ((61-58*T+T*T+600*C-330*UTM_EP2)
           * pow(A, 6)/720)));

  return;
}

/**
 * Convert lat/long to UTM coords.
 * 위도/경도를 UTM 좌표로 변환
 * East Longitudes are positive, West longitudes are negative.
 * 동경은 양수, 서경은 음수
 * North latitudes are positive, South latitudes are negative
 * 북위는 양수, 남위는 음수
 * Lat and Long are in fractional degrees
 * 위도와 경도는 분수도
 * @param[out] gamma meridian convergence at point (degrees).
 * output : gamma > 점에서 감마 자오선 수렴(도)
 */ 
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone, double &gamma)
{
  int zone;
  bool northp;
  double k_unused;
  GeographicLib::UTMUPS::Forward(Lat, Long, zone, northp, UTMEasting, UTMNorthing, gamma,
                                 k_unused, GeographicLib::UTMUPS::zonespec::MATCH);
  GeographicLib::MGRS::Forward(zone, northp, UTMEasting, UTMNorthing, -1, UTMZone);
}

/**
 * Convert lat/long to UTM coords.
 *  위도/경도를 UTM 좌표로 변환
 * East Longitudes are positive, West longitudes are negative.
 * 동경은 양수, 서경은 음수
 * North latitudes are positive, South latitudes are negative
 * 북위는 양수, 남위는 음수
 * Lat and Long are in fractional degrees
 * 위도와 경도는 분수도
 */
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone)
{
  double gamma = 0.0;
  LLtoUTM(Lat, Long, UTMNorthing, UTMEasting, UTMZone, gamma);
}

/**
 * Converts UTM coords to lat/long.
 * UTM 좌표를 위도/경도로 변환
 * East Longitudes are positive, West longitudes are negative.
 * 동경은 양수, 서경은 음수
 * North latitudes are positive, South latitudes are negative
 * 북위는 양수, 남위는 음수
 * Lat and Long are in fractional degrees
 * 위도와 경도는 분수도
 *
 * @param[out] gamma meridian convergence at point (degrees).
 * output : gamma > 점에서 감마 자오선 수렴(도)
 */
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const std::string &UTMZone, double& Lat, double& Long,
                           double& /*gamma*/)
{
  int zone;
  bool northp;
  double x_unused;
  double y_unused;
  int prec_unused;
  GeographicLib::MGRS::Reverse(UTMZone, zone, northp, x_unused, y_unused, prec_unused, true);
  GeographicLib::UTMUPS::Reverse(zone, northp, UTMEasting, UTMNorthing, Lat, Long);
}

/**
 * Converts UTM coords to lat/long.
 * UTM 좌표를 위도/경도로 변환
 * East Longitudes are positive, West longitudes are negative.
 * 동경은 양수, 서경은 음수
 * North latitudes are positive, South latitudes are negative
 * 북위는 양수, 남위는 음수
 * Lat and Long are in fractional degrees
 * 위도와 경도는 분수도
 */
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const std::string &UTMZone, double& Lat, double& Long)
{
  double gamma;
  UTMtoLL(UTMNorthing, UTMEasting, UTMZone, Lat, Long, gamma);
}

}  // namespace NavsatConversions
}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_NAVSAT_CONVERSIONS_H
