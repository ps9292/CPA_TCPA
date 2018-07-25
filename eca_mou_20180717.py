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

'''
------------------------------------------------------------------
 ECA + TCPA
 (수정)2018.07.17 MoU 모델적용
 (수정)2018.07.05 애니메이션 생성
 (수정)2018.07.04 정지선박 필터링, 충돌위험도 판단 알고리즘 구체화
 (수정)2018.06.28 상대선 eca_time 뒤 위치 계산
 (수정)2018.06.28 필요함수 설정  
 (초안)2018.06.26
 Hokun Jeon | 전호군
 hkjeon@kiost.ac.kr
 Marine Security & Safety Research Center | 해양방위안전연구센터
 Korea Institute of Ocean Science & Technology | 한국해양과학기술원
------------------------------------------------------------------
'''


''' 
-----------------------------------
Step1) 본선과 상대선의 기본정보 입력
-----------------------------------
'''
## 본선의 기본정보
os_lon=128.95;   # 본선위치 x    
os_lat=35.00;    # 본선위치 y       
os_spd=15;    # 본선속도 knot      마일(')/hr 단위
os_co=90       # 본선침로 도(°)
os_ant2bow=200 # 본선안테나로부터 본선선수까지  미터단위

### 본선의 정지
#os_lon=128.95;   # 본선위치 x    
#os_lat=35.00;    # 본선위치 y       
#os_spd=0;    # 본선속도 knot      마일(')/hr 단위
#os_co=90       # 본선침로 도(°)
#os_ant2bow=200 # 본선안테나로부터 본선선수까지  미터단위

### 상대선 조건 1    좌현교차(선수지남)
#ts_lon=129.03;   # 상대선위치 x
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
#ts_spd=25  # 상대선속도 knot    분(')/hr 단위
#ts_co=45   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터

### 상대선 조건 4  좌현교차(선미지남)
#ts_lon=129.015;   # 상대선위치 x
#ts_lat=35.05;    # 상대선위치 y
#ts_spd=10;  # 상대선속도 knot    분(')/hr 단위
#ts_co=180;   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터
##
### 상대선 조건 5   마주침(아슬하게 피함)
#ts_lon=129.05;   # 상대선위치 x
#ts_lat=35.005;    # 상대선위치 y
#ts_spd=15;  # 상대선속도 knot    분(')/hr 단위
#ts_co=270;   # 상대선침로 도(°)
#ts_ant2bow=100; # 상대선안테나로부터 상대선선수까지 거리 미터

# 상대선 조건 5   정지선박
ts_lon=129.05;   # 상대선위치 x
ts_lat=35.005;    # 상대선위치 y
ts_spd=0;  # 상대선속도 knot    분(')/hr 단위
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
Step4-1) 테스트용 시계열 데이터 제작
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


lon_r=ts_spd/60*numpy.sind(ts_co)-os_spd/60*numpy.sind(os_co)  #상대백터 경도  도(°) 단위
lat_r=ts_spd/60*numpy.cosd(ts_co)-os_spd/60*numpy.cosd(os_co)  #상대백터 위도  도(°) 단위
rv=math.sqrt((lon_r)**2+(lat_r)**2)           #상대백터 크기  도(°) 단위



' TCPA, DCPA 계산 알고리즘 - 한국해양대 제공본 수정 ----------------------------'
dist_deg=[math.sqrt((ts_lon_bow[i]-os_lon_bow[i])**2+(ts_lat_bow[i]-os_lat_bow[i])**2) for i in range(0,time_limit)]  # 본선과 상대선의 거리 deg 단위
dist_mile=[dist_deg[i]*60 for i in range(0,time_limit)]
lon_r=ts_spd/60*numpy.sind(ts_co)-os_spd/60*numpy.sind(os_co)  #상대백터 경도  도(°) 단위
lat_r=ts_spd/60*numpy.cosd(ts_co)-os_spd/60*numpy.cosd(os_co)  #상대백터 위도  도(°) 단위
rv=math.sqrt((lon_r)**2+(lat_r)**2)           #상대백터 크기  도(°) 단위
tcpa_hr=[((os_lon_bow[i]-ts_lon_bow[i])*lon_r+(os_lat_bow[i]-ts_lat_bow[i])*lat_r)/rv**2 for i in range(0,time_limit)]   # TCPA 시(hr) 단위
tcpa_min=[tcpa_hr[i]*60 for i in range(0,time_limit)]                     # TCPA 분(min) 단위
dcpa_mi=[math.sqrt(abs(((dist_mile[i])**2)-((rv*60*tcpa_hr[i])**2))) for i in range(0,time_limit)]   # DCPA 거리 mile단위


' TCPA후 본선과 상대선 선수위치 시계열데이터 '
os_lon_tcpa=[os_lon_bow[i]+os_spd/(60*60*60)*numpy.sind(os_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 본선의 tcpa(sec) 뒤 경도
os_lat_tcpa=[os_lat_bow[i]+os_spd/(60*60*60)*numpy.cosd(os_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 본선의 tcpa(sec) 뒤 위도 
ts_lon_tcpa=[ts_lon_bow[i]+ts_spd/(60*60*60)*numpy.sind(ts_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 경도
ts_lat_tcpa=[ts_lat_bow[i]+ts_spd/(60*60*60)*numpy.cosd(ts_co)*(tcpa_min[i]*60) for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 위도 


' TCPA후 본선 선수위치와 상대선 선수위치 간의 상대방위 계산 예) 앞 0 뒤 180 좌 270 우 90 '
rel_brg_tcpa=[rel_brg([os_lon_tcpa[i],os_lat_tcpa[i]],[ts_lon_tcpa[i],ts_lat_tcpa[i]]) for i in range(0,time_limit)]



'''
----------------------------------
Step4-2) MoU et. al. 모델적용
----------------------------------
'''

diff_ang=abs(os_co-ts_co)
if diff_ang>=180:
    diff_ang=360-max([os_co,ts_co])+min([os_co,ts_co])

if diff_ang>=0 and diff_ang<=60: # overtaking
    f_ang=1
elif diff_ang>60 and diff_ang<=150: #crossing
    f_ang=8.5
elif diff_ang>150 and diff_ang<=180: # head-on
    f_ang=2.34
    
   
cr=[math.exp(-tcpa_min[i]/10)*math.exp(-abs(dcpa_mi[i]))*f_ang for i in range(0,time_limit)] # 타선의 tcpa(sec) 뒤 위도 



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
        if ts_spd>=0 and ts_spd<=2: # 타선박이 정박중이거나 표류 중(0~2kt)일때
            if (rel_brg_now[i]<=-90 or rel_brg_now[i]>=90) or dcpa_mi[i]>0.35: # 타선박이 본선 정횡후에 위치하거나 dcpa 0.3 mile 초과시
                # 0.3mile 이유 :  부산북항통항선박 200m*2.85/1852m=0.307mile
                eca_color.append('NaN')  # 색지정하지 않음.
            else: # 타선박이 본선 정횡전에 위치하고 dcpa 0.2 mile 이하
                if tcpa_min[i]>5 and tcpa_min[i]<=10: # 5분<tcpa<=10분
                    eca_color.append('y')  # 노랑
                elif tcpa_min[i]>2 and tcpa_min[i]<=5: # 2분<tcpa<=5분
                    eca_color.append('C1') # 주황
                elif tcpa_min[i]>=0 and tcpa_min[i]<=2: # 0분<tcpa<=2분
                    eca_color.append('r') # 빨강
                elif tcpa_min[i]<0 or tcpa_min[i]>10: # tcpa<0분    
                    eca_color.append('NaN')  # 색지정하지 않음.

        elif ts_spd>2: # 타선박이 항해 중(2kt 이상)
             if dcpa_mi[i]>0.5: # dcpa 0.5 mile 초과
                eca_color.append('NaN')  # 색지정하지 않음.
             else:
                if tcpa_min[i]>5 and tcpa_min[i]<=10: # 5분<tcpa<=10분
                    eca_color.append('y')  # 노랑
                elif tcpa_min[i]>2 and tcpa_min[i]<=5: # 2분<tcpa<=5분
                    eca_color.append('C1') # 주황
                elif tcpa_min[i]>=0 and tcpa_min[i]<=2: # 0분<tcpa<=2분
                    eca_color.append('r') # 빨강
                elif tcpa_min[i]<0 or tcpa_min[i]>10: # tcpa<0분    
                    eca_color.append('NaN')  # 색지정하지 않음.           
    
                 
                            

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
##        
#
'''
-----------------------------------
Step6) 애니메이션 생성
-----------------------------------
'''
    
sp_intval=18    
ani_range=int(1800/sp_intval)

import matplotlib.pyplot as plt
    
for i in range(0,ani_range):    
    fig=plt.figure(1)
    plt.axes(aspect='equal',xlim =(128.90,129.13), ylim = (34.90,35.10))
    plt.plot(os_lon_now[i*sp_intval], os_lat_now[i*sp_intval], 'ob')
    plt.plot(ts_lon_now[i*sp_intval], ts_lat_now[i*sp_intval], marker='o', color=eca_color[i*sp_intval])    
    plt.text(129.06, 35.08, 'time_stmp', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.08, time_stamp[i*sp_intval], horizontalalignment='center', verticalalignment='center')
    plt.text(129.06, 35.07, 'TCPA(min)', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.07, round(tcpa_min[i*sp_intval],2), horizontalalignment='center', verticalalignment='center')  
    plt.text(129.06, 35.06, 'DCPA(mi)', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.06, round(dcpa_mi[i*sp_intval],2), horizontalalignment='center', verticalalignment='center')    
    plt.text(129.06, 35.05, 'DIST(mi)', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.05, round(dist_mile[i*sp_intval],2), horizontalalignment='center', verticalalignment='center')        
    plt.text(129.06, 35.04, 'REL.BRG', horizontalalignment='center', verticalalignment='center')    
    plt.text(129.10, 35.04, round(rel_brg_now[i*sp_intval],1), horizontalalignment='center', verticalalignment='center')    
    plt.pause(0.01)
    plt.clf()
    plt.cla()        
plt.show()