# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 20:36:26 2018

@author: Jeong
"""

# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

'''---------------------------------------
 Emergency Collision Alarm(ECA)
 (수정)2018.07.04 정지선박 필터링
 (수정)2018.06.28 상대선 eca_time 뒤 위치 계산
 (수정)2018.06.28 필요함수 설정  
 (초안)2018.06.26
 KIOST LaSOMS 전호군
---------------------------------------'''


''' 
-----------------------------------
Step1) 본선과 상대선의 기본정보 입력
-----------------------------------
#'''
### 본선의 기본정보
#os_lon=128.95;   # 본선위치 x    
#os_lat=35.00;    # 본선위치 y       
#os_spd=15;    # 본선속도 knot      마일(')/hr 단위
#os_co=90       # 본선침로 도(°)
#os_ant2bow=200 # 본선안테나로부터 본선선수까지  미터단위

## 본선의 정지
os_lon=128.95;   # 본선위치 x    
os_lat=35.00;    # 본선위치 y       
os_spd=0;    # 본선속도 knot      마일(')/hr 단위
os_co=90       # 본선침로 도(°)
os_ant2bow=200 # 본선안테나로부터 본선선수까지  미터단위

### 상대선 조건 1    좌현교차(선수지남)
#ts_lon=129.05;   # 상대선위치 x
#ts_lat=35.10;    # 상대선위치 y
#ts_spd=20;  # 상대선속도 knot    마일(')/hr 단위
#ts_co=180;   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리  미터단위

#### 상대선 조건 2   마주침(정면충돌)
#ts_lon=129.10;   # 상대선위치 x
#ts_lat=35.00;    # 상대선위치 y
#ts_spd=10;  # 상대선속도 knot    분(')/hr 단위
#ts_co=270;   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터
#
### 상대선 조건 3   추월
#ts_lon=128.90   # 상대선위치 x
#ts_lat=35.00    # 상대선위치 y
#ts_spd=30  # 상대선속도 knot    분(')/hr 단위
#ts_co=90   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터

### 상대선 조건 3   대각선추월
#ts_lon=128.95   # 상대선위치 x
#ts_lat=34.95    # 상대선위치 y
#ts_spd=40  # 상대선속도 knot    분(')/hr 단위
#ts_co=45   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터

### 상대선 조건 4   우현교차(선미지남)
#ts_lon=129.05;   # 상대선위치 x
#ts_lat=35.05;    # 상대선위치 y
#ts_spd=10;  # 상대선속도 knot    분(')/hr 단위
#ts_co=180;   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터
##
### 상대선 조건 5   상대선박
#ts_lon=129.05;   # 상대선위치 x
#ts_lat=35.005;    # 상대선위치 y
#ts_spd=15;  # 상대선속도 knot    분(')/hr 단위
#ts_co=270;   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터

# 상대선 조건 5   정지선박
ts_lon=129.05;   # 상대선위치 x
ts_lat=35.005;    # 상대선위치 y
ts_spd=15;  # 상대선속도 knot    분(')/hr 단위
ts_co=270;   # 상대선침로 도(°)
ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터

if os_spd==0:
    os_spd=0.00000000000000000000000000001
if ts_spd==0:
    ts_spd=0.00000000000000000000000000001
    
''' 
-----------------------------------
Step2) ECA 기본정보 입력
-----------------------------------
'''

'ECA 기본정보'  # Emergency Collision Alarm, ECA 긴급충돌알람 
eca_time=60 # 몇초 뒤 결과를 예측할지 정함
eca_brg=5       # 경계 좌우현 방위
time_limit=30*60  # 몇 초까지 계산할지 정함
time_stamp=list(range(0,time_limit))


''' 
-----------------------------------
Step3) 필요한 함수 미리 정의
-----------------------------------
'''

'삼각함수 정의'
import numpy
numpy.cosd = lambda x : numpy.cos(numpy.deg2rad(x))
numpy.sind = lambda x : numpy.sin(numpy.deg2rad(x))
numpy.cosd = lambda x : numpy.cos(numpy.deg2rad(x))
numpy.tand = lambda x : numpy.tan(numpy.deg2rad(x))
numpy.secd = lambda x : 1/numpy.sin(numpy.deg2rad(x))


'자이로 방위를 평면방위로 변환'     # 예) 본선컴파스 방위 90도 = 평면방위 0
def gyro2deg(gyro):   
    deg=-gyro+90
    if deg<360:
        deg=(deg+360)%360
    elif deg>360:
        deg=deg%360
    if deg==360:
        deg=0
    return deg


'초당 경위도 이동거리 계산'
def d_sec(co, spd):
    # 첫번째 60 : mile/hr를 deg/hr로 변환.
    # 두번째 60 : deg/hr를 deg/min로 변환.
    # 세번쨰 60 : deg/min를 deg/sec로 변환.
    lon_d_sec=spd/(60*60*60)*numpy.cosd(gyro2deg(co))   #초당 경도 이동거리 (도, deg)
    lat_d_sec=spd/(60*60*60)*numpy.sind(gyro2deg(co))   #초당 위도 이동거리 (도, deg)
    return lon_d_sec, lat_d_sec


'현재 선수위치 계산'
def psn_bow(lon_now,lat_now, ant2bow, co):    # 경도, 위도, 안테나로부터 선수bow까지 거리(m), 침로
    ant2bow=ant2bow/(1852*60)         # meter로 입력받은 길이(m)를 도(deg) 단위로 변환
    lon_psn_bow= lon_now+ant2bow*numpy.cosd(gyro2deg(co)) # 선수위치 보정 경도.  도(deg) 단위
    lat_psn_bow= lat_now+ant2bow*numpy.sind(gyro2deg(co)) # 선수위치 보정 위도.  도(deg) 단위
    return lon_psn_bow, lat_psn_bow


'탐지 거리 '
def eca_dist(spd,eca_time):   # 속도, 몇 초(eca_time)
    dist=spd/(60*60*60)*eca_time   # 도(deg)단위 
    return dist


' 몇 초뒤 위치 '
def psn_eca(lon_bow,lat_bow,co,spd,eca_time):   # 선수위치 경도, 선수위치 위도, 침로, 선속, eca시간
    eca_lon=lon_bow+spd/(60*60*60)*numpy.cosd(gyro2deg(co))*eca_time  # 몇 초뒤 선수위치 경도. 도(deg)단위
    eca_lat=lat_bow+spd/(60*60*60)*numpy.sind(gyro2deg(co))*eca_time  # 몇 초뒤 선수위치 위도. 도(deg)단위
    return eca_lon, eca_lat



''' 
-----------------------------------
Step4) 테스트용 시계열 데이터 제작
-----------------------------------
'''

' 선수위치 이동궤적 테스트용 시계열 데이터 제작 '
# 본선데이터
os_lon_now=[os_lon+i*d_sec(os_co,os_spd)[0] for i in range(0,time_limit)] # 출발지를 기준으로 매초 안테나 위치 경도
os_lat_now=[os_lat+i*d_sec(os_co,os_spd)[1] for i in range(0,time_limit)] # 출발지를 기준으로 매초 안테나 위치 위도
os_lon_bow=psn_bow(os_lon_now,os_lat_now,os_ant2bow,os_co)[0]  # 각 시점의 선수위치 경도
os_lat_bow=psn_bow(os_lon_now,os_lat_now,os_ant2bow,os_co)[1]  # 각 시점의 선수위치 경도

# 타선데이터
ts_lon_now=[ts_lon+i*d_sec(ts_co,ts_spd)[0] for i in range(0,time_limit)] # 출발지를 기준으로 매초 안테나 위치 경도
ts_lat_now=[ts_lat+i*d_sec(ts_co,ts_spd)[1] for i in range(0,time_limit)] # 출발지를 기준으로 매초 안테나 위치 위도
ts_lon_bow=psn_bow(ts_lon_now,ts_lat_now,ts_ant2bow,ts_co)[0]   # 각 시점의 선수위치 경도
ts_lat_bow=psn_bow(ts_lon_now,ts_lat_now,ts_ant2bow,ts_co)[1]   # 각 시점의 선수위치 위도
ts_lon_eca=psn_eca(ts_lon_bow,ts_lat_bow,ts_co,ts_spd,eca_time)[0]   # 각 시점에서 eca_time(sec)가 지난 후 선수위치 경도
ts_lat_eca=psn_eca(ts_lon_bow,ts_lat_bow,ts_co,ts_spd,eca_time)[1]   # 각 시점에서 eca_time(sec)가 지난 후 선수위치 위도


' ECA_TIME뒤 본선과 상대선의 선수위치 시계열데이터 '
ts_lon_eca=ts_lon_bow+ts_spd/(60*60*60)*numpy.sind(ts_co)*eca_time # 타선의 eca_time(sec) 뒤 경도
ts_lat_eca=ts_lat_bow+ts_spd/(60*60*60)*numpy.cosd(ts_co)*eca_time # 타선의 eca_time(sec) 뒤 위도

import math
' 두 점의 상대방위를 구하는 함수  '
def rel_brg(pointA,pointB):
    pointA=tuple(pointA)
    pointB=tuple(pointB)

    lat1 = math.radians(pointA[1])
    lat2 = math.radians(pointB[1])
    diffLong = math.radians(pointB[0] - pointA[0])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
        * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180° to + 180° which is not what we want for a compass bearing
    # The solution is to normalize the initial bearing as shown below
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    
    rel_bearing=compass_bearing-os_co
#    if rel_bearing<0:
#        rel_bearing=(rel_bearing+360)%360
#    elif rel_bearing>=360:
#        rel_bearing=(rel_bearing-360)%360
    return rel_bearing

' 본선의 현재 선수위치와 상대선의 ECA_TIME뒤 선수위치 간의 상대방위 계산 예) 앞 0 뒤 180 좌 270 우 90 '
rel_brg_eca=[rel_brg([os_lon_bow[i],os_lat_bow[i]],[ts_lon_eca[i],ts_lat_eca[i]]) for i in range(0,time_limit)]

' 현재 본선 선수위치와 상대선 선수위치 간의 상대방위 계산 예) 앞 0 뒤 180 좌 270 우 90 '
rel_brg_now=[rel_brg([os_lon_bow[i],os_lat_bow[i]],[ts_lon_bow[i],ts_lat_bow[i]]) for i in range(0,time_limit)]    

' 본선의 현재 선수위치와 상대선의 ECA_TIME뒤 선수위치 간의 거리 계산 '
dist_eca=[math.sqrt((os_lon_bow[i]-ts_lon_eca[i])**2+(os_lat_bow[i]-ts_lat_eca[i])**2) for i in range(0,time_limit)] # deg 단위


' TCPA, DCPA 계산 알고리즘 - 한국해양대 제공본 수정 ----------------------------'
dist_deg=[math.sqrt((ts_lon_bow[i]-os_lon_bow[i])**2+(ts_lat_bow[i]-os_lat_bow[i])**2) for i in range(0,time_limit)]  # 본선과 상대선의 거리 deg 단위
dist_mile=[dist_deg[i]*60 for i in range(0,time_limit)]
lon_r=ts_spd/60*numpy.sind(ts_co)-os_spd/60*numpy.sind(os_co)  #상대백터 경도  도(°) 단위
lat_r=ts_spd/60*numpy.cosd(ts_co)-os_spd/60*numpy.cosd(os_co)  #상대백터 위도  도(°) 단위
rv=math.sqrt((lon_r)**2+(lat_r)**2)           #상대백터 크기  도(°) 단위
tcpa_hr=[((os_lon_bow[i]-ts_lon_bow[i])*lon_r+(os_lat_bow[i]-ts_lat_bow[i])*lat_r)/rv**2 for i in range(0,time_limit)]   # TCPA 시(hr) 단위
tcpa_min=[tcpa_hr[i]*60 for i in range(0,time_limit)]                     # TCPA 분(min) 단위
dcpa_mi=[math.sqrt(((dist_mile[i])**2)-((rv*60*tcpa_hr[i])**2)) for i in range(0,time_limit)]   # DCPA 거리 mile단위


#' TCPA후 본선과 상대선 선수위치 시계열데이터 '
#os_lon_tcpa=[os_lon_bow[i]+os_spd/(60*60*60)*numpy.sind(os_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 본선의 tcpa(sec) 뒤 경도
#os_lat_tcpa=[os_lat_bow[i]+os_spd/(60*60*60)*numpy.cosd(os_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 본선의 tcpa(sec) 뒤 위도 
#ts_lon_tcpa=[ts_lon_bow[i]+ts_spd/(60*60*60)*numpy.sind(ts_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 경도
#ts_lat_tcpa=[ts_lat_bow[i]+ts_spd/(60*60*60)*numpy.cosd(ts_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 위도 

' TCPA후 본선과 상대선 선수위치 시계열데이터 '
os_lon_tcpa=[os_lon_bow[i]+os_spd/(60*60*60)*numpy.sind(os_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 본선의 tcpa(sec) 뒤 경도
os_lat_tcpa=[os_lat_bow[i]+os_spd/(60*60*60)*numpy.cosd(os_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 본선의 tcpa(sec) 뒤 위도 
ts_lon_tcpa=[ts_lon_bow[i]+ts_spd/(60*60*60)*numpy.sind(ts_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 경도
ts_lat_tcpa=[ts_lat_bow[i]+ts_spd/(60*60*60)*numpy.cosd(ts_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 위도 


' TCPA후 본선 선수위치와 상대선 선수위치 간의 상대방위 계산 예) 앞 0 뒤 180 좌 270 우 90 '
rel_brg_tcpa=[rel_brg([os_lon_tcpa[i],os_lat_tcpa[i]],[ts_lon_tcpa[i],ts_lat_tcpa[i]]) for i in range(0,time_limit)]


'''
-----------------------------------
Step5) 충돌위험상황 인지하기
    1차 1분뒤 충돌할 급박한 상황을 인지한다.
    2차 TCPA, DCPA를 통해 1차 이외의 상황을 인지한다.
-----------------------------------
'''


eca_color=list()
for i in range(0,time_limit):
    if (rel_brg_eca[i]>-eca_brg and rel_brg_eca[i]<eca_brg) and dist_eca[i]<eca_dist(os_spd,eca_time): # 경계를 위해 설정한 상대방위(eca_brg)와 거리(eca_dist) 안에 있으면
        eca_color.append('r') # 위험 red
        
    else: # ECA에 해당하지 않는다면
        '타 선박이 정박중이거나 표류 중일 떄'
        if ts_spd>=0 and ts_spd<=2: # 타선박이 정박중이거나 표류 중일 때. (0~2kt)
            if tcpa_min[i]>=0: # 최근접점을 만날 예정
                if dcpa_mi[i]>0.3: # dcpa값이 0.3 이상으로 안전
                    eca_color.append('k')    
                elif dcpa_mi[i]>0.2 and dcpa_mi[i]<=0.3: # dcpa값이 0.2~0.3으로 주의필요.
                    if rel_brg_now[i]<=-90 or rel_brg_now[i]>=90: # 타선박이 본선 정횡후에 위치한 경우
                        eca_color.append('k')
                    elif (rel_brg_now[i]>-90 and rel_brg_now[i]<-45) or (rel_brg_now[i]>45 and rel_brg_now[i]<90) :   # 정선수중심 45~90도
                        if dist_mile[i]>2:           # 거리 2마일 이상 
                            eca_color.append('k')
                        elif dist_mile[i]>1 and dist_mile[i]<=2: # 거리 1~2마일
                            eca_color.append('C1')
                        elif dist_mile[i]>=0 and dist_mile[i]<=1: #거리 0~1마일
                            eca_color.append('r')

                    elif (rel_brg_now[i]>-45 and rel_brg_now[i]<45): # 정선수중심 45도
                        if dist_mile[i]>3: # 거리 3마일 이상
                            eca_color.append('k')
                        elif dist_mile[i]>2 and dist_mile[i]<=3: #거리 2~3마일
                            eca_color.append('C1')
                        elif dist_mile[i]>=0 and dist_mile[i]<=2: #거리 0~2마일
                            eca_color.append('r')    
                            
                elif dcpa_mi[i]>=0 and dcpa_mi[i]<=0.2: # dcpa값이 0.2이하로 위험.
                    if rel_brg_now[i]<=-90 or rel_brg_now>=90: # 타선박이 본선의 정횡후에 위치한 경우
                        eca_color.append('k')
                    elif (rel_brg_now[i]>-90 and rel_brg_now[i]<-45) or (rel_brg_now[i]>45 and rel_brg_now[i]<90):   # 정선수중심 45~90도
                        if dist_mile[i]>3: 
                            eca_color.append('k')
                        elif dist_mile[i]>2 and dist_mile[i]<=3:
                            eca_color.append('C1')
                        elif dist_mile[i]>=0 and dist_mile[i]<=2:
                            eca_color.append('r')
                            
                    elif (rel_brg_now[i]>-45 and rel_brg_now[i]<45):   # 정선수중심 45도 영역
                        if dist_mile[i]>4:
                            eca_color.append('k')
                        elif dist_mile[i]>3 and dist_mile[i]<=4:
                            eca_color.append('C1')
                        elif dist_mile[i]>=0 and dist_mile[i]<=3:
                            eca_color.append('r')     
                            
            elif tcpa_min[i]<0:  # 최근접점을 지나침.
                eca_color.append('k')

        elif ts_spd>2: # 타선박이 항해하는 경우 (2kt 이상)
            if tcpa_min[i]>=0:  # 최근접점에서 만날 예정 tcpa_min>=0
                if dcpa_mi[i]>1: # dcpa값이 1마일 이상으로 안전
                    eca_color.append('k')    

                elif dcpa_mi[i]>0.5 and dcpa_mi[i]<=1: # dcpa값이 0.5~1마일으로 주의필요.
                    if tcpa_min[i]>15:  # tcpa>15분
                        eca_color.append('k')
                    elif tcpa_min[i]>5 and tcpa_min[i]<=15:    # tcpa 5~15분                    
                        '현재를 기준으로 한 상대방위 적용'
                        if rel_brg_now[i]<=-90 or rel_brg_now[i]>=90: # TCPA에서 타선박이 본선의 정횡후에 위치할 경우
                                eca_color.append('k')    
                        elif (rel_brg_now[i]>-90 and rel_brg_now[i]<=-45) or (rel_brg_now[i]>=45 and rel_brg_now[i]<90):   # # TCPA뒤 타선박이 본선의 정선수중심 45~90도
                            if dist_mile[i]>3:
                                eca_color.append('k')
                            elif dist_mile[i]<=3:
                                eca_color.append('C1')
                        elif (rel_brg_now[i]>-45 and rel_brg_now[i]<=-10) or (rel_brg_now[i]>=10 and rel_brg_now[i]<45):   # # TCPA뒤 타선박이 본선의 정선수중심 10~45도                                
                            if dist_mile[i]>6:
                                eca_color.append('k')
                            elif dist_mile[i]<=6:
                                eca_color.append('C1')                       
                        elif (rel_brg_now[i]>-10 and rel_brg_now[i]<10):   # # TCPA에서 타선박이 본선의 정선수중심 10도 이내 영역
                            if dist_mile[i]>12:
                                eca_color.append('k')
                            elif dist_mile[i]<=12:
                                eca_color.append('C1')
                            
                    elif tcpa_min[i]>0 and tcpa_min[i]<=5:  # tcpa 0~5분
                        if rel_brg_now[i]<=-90 or rel_brg_now[i]>=90: # TCPA에서 타선박이 본선의 정횡후에 위치할 경우
                            if dist_mile[i]>1:
                                eca_color.append('k')                                
                            elif dist_mile[i]>0.5 and dist_mile[i]<=1:
                                eca_color.append('C1')
                            elif dist_mile[i]<=0.5:
                                eca_color.append('r')
                        elif (rel_brg_now[i]>-90 and rel_brg_now[i]<=-45) or (rel_brg_now[i]>=45 and rel_brg_now[i]<90):   # # TCPA뒤 타선박이 본선의 정선수중심 45~90도
                            if dist_mile[i]>3:
                                eca_color.append('k')
                            elif dist_mile[i]>1 and dist_mile[i]<=3:
                                eca_color.append('C1')
                            elif dist_mile[i]<=1:
                                eca_color.append('r')
                        elif (rel_brg_now[i]>-45 and rel_brg_now[i]<=-10) or (rel_brg_now[i]>=10 and rel_brg_now[i]<45):   # # TCPA뒤 타선박이 본선의 정선수중심 10~45도                                
                            if dist_mile[i]>7:
                                eca_color.append('k')
                            elif dist_mile[i]>2 and dist_mile[i]<=7:
                                eca_color.append('C1')
                            elif dist_mile[i]<=2:
                                eca_color.append('r')                            
                        elif (rel_brg_now[i]>-10 and rel_brg_now[i]<10):   # # TCPA에서 타선박이 본선의 정선수중심 10도 이내 영역
                            if dist_mile[i]>=10:
                                eca_color.append('C1')
                            elif dist_mile[i]<10:
                                eca_color.append('r')                         
                        
                elif dcpa_mi[i]<=0.5: # dcpa값이 0.5이하로 위험.
                    if tcpa_min[i]>15:  # tcpa>15분 또는 tcpa<0
                        eca_color.append('k')
                    elif tcpa_min[i]>5 and tcpa_min[i]<=15:    # tcpa 5~15분                    
                        '현재를 기준으로 한 상대방위 적용'                        
                        if rel_brg_now[i]<=-90 or rel_brg_now[i]>=90: # TCPA에서 타선박이 본선의 정횡후에 위치할 경우
                            if dist_mile[i]>1:
                                eca_color.append('k')                                
                            elif dist_mile[i]>0.5 and dist_mile[i]<=1:
                                eca_color.append('C1')
                            elif dist_mile[i]<=0.5:
                                eca_color.append('r')
                        elif (rel_brg_now[i]>-90 and rel_brg_now[i]<=-45) or (rel_brg_now[i]>=45 and rel_brg_now[i]<90):   # # TCPA뒤 타선박이 본선의 정선수중심 45~90도
                            if dist_mile[i]>3:
                                eca_color.append('k')
                            elif dist_mile[i]<=3:
                                eca_color.append('C1')
                        elif (rel_brg_now[i]>-45 and rel_brg_now[i]<=-10) or (rel_brg_now[i]>=10 and rel_brg_now[i]<45):   # # TCPA뒤 타선박이 본선의 정선수중심 10~45도                                
                            if dist_mile[i]>9:
                                eca_color.append('k')
                            elif dist_mile[i]<=9:
                                eca_color.append('C1')
                        elif (rel_brg_now[i]>-10 and rel_brg_now[i]<10):   # # TCPA에서 타선박이 본선의 정선수중심 10도 이내 영역
                            if dist_mile[i]>15:
                                eca_color.append('k')
                            elif dist_mile[i]<=15:
                                eca_color.append('C1')
                    elif tcpa_min[i]>=0 and tcpa_min[i]<=5:    # tcpa 0~5분                    
                        '현재를 기준으로 한 상대방위 적용'                                                        
                        if rel_brg_now[i]<=-120 or rel_brg_now[i]>=120: # TCPA에서 타선박이 본선 정선수중심 120도 이외 위치할 경우
                            if dist_mile[i]>1:
                                eca_color.append('k')                                
                            elif dist_mile[i]>0.5 and dist_mile[i]<=1:
                                eca_color.append('C1')
                            elif dist_mile[i]<=0.5:
                                eca_color.append('r')
                        elif (rel_brg_now[i]>-120 and rel_brg_now[i]<=-45) or (rel_brg_now[i]>=45 and rel_brg_now[i]<120):   # # TCPA뒤 타선박이 본선의 정선수중심 45~90도
                            if dist_mile[i]>3:
                                eca_color.append('k')
                            elif dist_mile[i]>1 and dist_mile[i]<=3:
                                eca_color.append('C1')
                            elif dist_mile[i]<=1:
                                eca_color.append('r')
                        elif (rel_brg_now[i]>-45 and rel_brg_now[i]<=-10) or (rel_brg_now[i]>=10 and rel_brg_now[i]<45):   # # TCPA뒤 타선박이 본선의 정선수중심 10~45도                                
                            if dist_mile[i]>10:
                                eca_color.append('k')
                            elif dist_mile[i]>4 and dist_mile[i]<=10:
                                eca_color.append('C1')
                            elif dist_mile[i]<=4:
                                eca_color.append('r')                            
                        elif (rel_brg_now[i]>-10 and rel_brg_now[i]<10):   # # TCPA에서 타선박이 본선의 정선수중심 10도 이내 영역
                            if dist_mile[i]>15:
                                eca_color.append('k')
                            elif dist_mile[i]>10 and dist_mile[i]<=15:
                                eca_color.append('C1')
                            elif dist_mile[i]<=10:
                                eca_color.append('r')                                 

            else: # 최근접점을 지난 후
                eca_color.append('k')                          

    
#        elif ts_spd>2: # 타선박이 항해하는 경우 (2kt 이상)
#            if tcpa_min[i]>=0:  # 최근접점에서 만날 예정 tcpa_min>=0
#                if dcpa_mi[i]>1: # dcpa값이 1마일 이상으로 안전
#                    eca_color.append('k')    
#
#                elif dcpa_mi[i]>0.5 and dcpa_mi[i]<=1: # dcpa값이 0.5~1마일으로 주의필요.
#                    if tcpa_min[i]>15 or tcpa_min[i]<0:  # tcpa>15분 또는 tcpa<0
#                        eca_color.append('k')
#                    elif tcpa_min[i]>5 and tcpa_min[i]<=15:    # tcpa 5~15분                    
#                        '현재를 기준으로 한 상대방위 적용'
#                        if rel_brg_tcpa<=-90 or rel_brg_tcpa>=90: # TCPA에서 타선박이 본선의 정횡후에 위치할 경우
#                            eca_color.append('k')
#                        elif (rel_brg_tcpa>-90 and rel_brg_tcpa<-45) or (rel_brg_tcpa>45 and rel_brg_tcpa<90):   # # TCPA뒤 타선박이 본선의 정선수중심 45~90도
#                            eca_color.append('o')
#                        elif (rel_brg_tcpa>-45 and rel_brg_tcpa<45):   # # TCPA에서 타선박이 본선의 정선수중심 45도 이내 영역
#                            eca_color.append('r')   
#                            
#                    elif tcpa_min[i]>0 and tcpa_min[i]<=5:  # tcpa 0~5분
#                        if rel_brg_tcpa<=-90 or rel_brg_tcpa>=90: # TCPA에서 타선박이 본선의 정횡후에 위치할 경우
#                            eca_color.append('k')
#                        elif (rel_brg_tcpa>-90 and rel_brg_tcpa<=-60) or (rel_brg_tcpa>=60 and rel_brg_tcpa<90):   # # TCPA뒤 타선박이 본선의 정선수중심 60~90도
#                            eca_color.append('o')
#                        elif (rel_brg_tcpa>-60 and rel_brg_tcpa<60):   # # TCPA에서 타선박이 본선의 정선수중심 60도 이내 영역
#                            eca_color.append('r')                               
#
#                        
#                elif dcpa_mi[i]>=0 and dcpa_mi[i]<=0.5: # dcpa값이 0.5이하로 위험.
#                        if rel_brg_tcpa<=-120 or rel_brg_tcpa>=120: # TCPA에서 타선박이 본선의 정선수중심 120 뒤에 위치할 경우
#                            eca_color.append('k')
#                        elif (rel_brg_tcpa>-120 and rel_brg_tcpa<-90) or (rel_brg_tcpa>90 and rel_brg_tcpa<120):   # TCPA뒤 타선박이 본선의 정선수중심 90~120도
#                            eca_color.append('o')
#                        elif (rel_brg_tcpa>-90 and rel_brg_tcpa<90):   # TCPA에서 타선박이 본선의 정선수중심 45도 이내 영역
#                            eca_color.append('r')                   
#
#            else: # 최근접점을 지난 후
#                eca_color.append('k')                          
      
                            

import pandas as pd
cal_table=pd.DataFrame({'time_stamp':time_stamp,
                        'dist_mile':dist_mile,
                        'rel_brg_now':rel_brg_now,
                        'tcpa_min':tcpa_min,
                        'dcpa_mi':dcpa_mi,
                        'os_lon_tcpa':os_lon_tcpa,
                        'os_lat_tcpa':os_lat_tcpa,
                        'rel_brg_tcpa':rel_brg_tcpa,
                        'eca_color':eca_color})
        

    
sp_intval=18    
ani_range=int(1800/sp_intval)

import matplotlib.pyplot as plt
    
for i in range(0,ani_range):    
    fig=plt.figure(1)
    plt.axes(aspect='equal',xlim =(128.90,129.13), ylim = (34.97,35.13))
    plt.plot(os_lon_now[i*sp_intval], os_lat_now[i*sp_intval], 'ob')
    plt.plot(ts_lon_now[i*sp_intval], ts_lat_now[i*sp_intval], marker='o', color=eca_color[i*sp_intval])    
    plt.text(129.06, 35.12, 'time_stmp', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.12, time_stamp[i*sp_intval], horizontalalignment='center', verticalalignment='center')
    plt.text(129.06, 35.11, 'TCPA(min)', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.11, round(tcpa_min[i*sp_intval],2), horizontalalignment='center', verticalalignment='center')  
    plt.text(129.06, 35.10, 'DCPA(mi)', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.10, round(dcpa_mi[i*sp_intval],2), horizontalalignment='center', verticalalignment='center')    
    plt.text(129.06, 35.09, 'DIST(mi)', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.09, round(dist_mile[i*sp_intval],2), horizontalalignment='center', verticalalignment='center')        
    plt.text(129.06, 35.08, 'REL.BRG', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.08, round(rel_brg_now[i*sp_intval],1), horizontalalignment='center', verticalalignment='center')    
    plt.pause(0.01)
    plt.clf()
    plt.cla()
plt.show()

#plt.figure(1)
#plt.axes(aspect='equal',xlim =(128.90,129.13), ylim = (34.97,35.13))
#plt.plot(os_lon_now[i], os_lat_now[i], 'ob')
#plt.plot(ts_lon_now[i], ts_lat_now[i], marker='o', color=eca_color[i])


#
#   
#    
#for i in [0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800]:
#    
#    import matplotlib.pyplot as plt
#    fig = plt.figure(i)
#    ax  = plt.axes(aspect='equal',xlim =(128.90,129.13), ylim = (34.97,35.13))
#    
#    plt_time=i
#            
#    if os_spd>=0 and os_spd<=2: 
#        plt.plot(os_lon_bow[plt_time],os_lat_bow[plt_time], 'ob')
#    elif os_spd>2:
#        plt.quiver(os_lon_bow[plt_time], os_lat_bow[plt_time], d_sec(os_co,os_spd)[0], d_sec(os_co,os_spd)[1], units='width',color='b')
#    
#    if ts_spd>=0 and ts_spd<=2: 
#        plt.plot(ts_lon_bow[plt_time],ts_lat_bow[plt_time], color=eca_color[plt_time], marker='o')
#    elif ts_spd>2:
#        plt.quiver(ts_lon_bow[plt_time], ts_lat_bow[plt_time], d_sec(ts_co,ts_spd)[0], d_sec(ts_co,ts_spd)[1], units='width',color=eca_color[plt_time])