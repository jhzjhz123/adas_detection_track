#include <deque>
#include "TTC.h"

template<typename Tqueue, typename Tobj>
void TTC::FetchDataFromQueue(Tqueue &queue, Tobj &tobj){
    if(queue.size() >= TTCTRACKINGFRAMES){
        this->FindObjectToUpdate(queue); 
        tobj = *(queue.end()-1);  //更新objttc值，主要是ttc的值
        queue.pop_front();
    }
}


template<typename Tqueue>
void TTC::FindObjectToUpdate(Tqueue &queue){
    auto itobj = (queue.end()-1); 
    for (auto i = itobj->begin(); i != itobj->end(); i++)
    {
        if ((i->second.objstate == 1)) //如果此id靠近，则计算前10帧，在内部，满足11帧全部跟踪到了，则计算ttc 
        {
            i->second.objttc = this->update(queue, i->first);
        }
    }
}


template<typename Tqueue, typename Tint>
ObjectTTC TTC::update(Tqueue &queue, Tint &frameid)
{   
    ObjectTTC objttc;

    //相当于首先计算纵坐标方向的ttc                 
    //std::cout << "objid: " << frameid << " ";
    double heights [TTCTRACKINGFRAMES] = { };
    double xr_sets [TTCTRACKINGFRAMES] = { };
    double xl_sets [TTCTRACKINGFRAMES] = { };
    double frameid_sets [TTCTRACKINGFRAMES] = { };
    double objnew_dis;
    double right_point;
    double left_point;
    for (int i = 0; i != TTCTRACKINGFRAMES; i++)
    {
        auto objt = queue.at(i).find(frameid);
        if (objt != queue.at(i).end())
        {
            object objtemp = objt->second;
            heights[i] = objtemp.size().height; //框高度，像素值      
            // heights[i] = objtemp.size().width; //框宽度，像素值 ，仅测试路标使用      
        
            xr_sets[i] = ((objtemp.worldposition().y) + (objtemp.worldposition().width / 2.)) * 1000; // y代表落地点的横向距离，左-右+,单位mm
            xl_sets[i] = ((objtemp.worldposition().y) - (objtemp.worldposition().width / 2.)) * 1000;
            // std::cout << "dis hengxiang: " << objtemp.worldposition().y <<" " <<  objtemp.worldposition().x << " " << objtemp.size().height << " " << objtemp.size().width << std::endl;
            frameid_sets[i] = objtemp.frame_index();
            
            if (i == (TTCTRACKINGFRAMES - 1))
            {   // std::cout <<"objnew_dis: " ;
                objnew_dis = objtemp.worldposition().x * 1000;
                //std::cout << objnew_dis << " " << std::endl; 
            }
        }
        else
        {
            objttc.ttc_existd = false;
            objttc.ttc_time = 88.8; //转化为s
            return objttc;
        }
    }

    double safe_disl =  (VEHICLEWIDTH * 0.25 + CAMERALEFT) * 1000.; //1.9 单位转化为mm
    double safe_disr = (-1.0) * (VEHICLEWIDTH * 1.25 - CAMERALEFT) * 1000.; //1350
    // double vehicle_speed = ??;
    // if (vehcle_speed != 0.0){ 
    if ((objnew_dis < 3000) && (xr_sets[TTCTRACKINGFRAMES -1] * xl_sets[TTCTRACKINGFRAMES -1]) > 0)
    {   if ((xr_sets[TTCTRACKINGFRAMES -1] > 0) && (xr_sets[TTCTRACKINGFRAMES -1] > safe_disl)){}
            //std::cout << "dis ok~" << std::endl;
        else if ((xl_sets[TTCTRACKINGFRAMES -1] < 0) && (xl_sets[TTCTRACKINGFRAMES -1] < safe_disr)){}
            //std::cout << "dis ok~" << std::endl;
        else
        {   
            //std::cout << "3 meters has objects,please slow down or parking!!" << std::endl;
            objttc.ttc_existd = true;
            objttc.ttc_time = 0.01; //转化为s
            return objttc;
        }
    }
// }
    std::vector<Point> points_height;
    cv::Point points_temph;
    // std::cout <<"heights: " << std::endl;
    float time_interval = static_cast<float>((frameid_sets[1] - frameid_sets[0]))/FRAMEFREQUENCY * 1000;
    for (int j = 0; j != (TTCTRACKINGFRAMES); j++)
    {
        // points_temph.x = ALPHATIME * (frameid_sets[j] - frameid_sets[0]); //横坐标轴时间，单位ms
        points_temph.x = j * time_interval;
        points_temph.y = heights[j];
        // std::cout << heights[h] << std::endl;
        points_height.push_back(points_temph);
    }

    cv::Vec4f fitheight;
    cv::fitLine(points_height, fitheight, CV_DIST_HUBER, 0, 0.01, 0.01);
    double k_height = fitheight[1] / fitheight[0];

    int frame_interval_mid = (frameid_sets[(TTCTRACKINGFRAMES - 1) / 2] - frameid_sets[0]);
    int frame_interval_end = (frameid_sets[TTCTRACKINGFRAMES -1] - frameid_sets[0]);

    double timeB = time_interval * frame_interval_mid; //ms
    double timeC = time_interval * frame_interval_end;

    double heightA = k_height * (0 - fitheight[2]) + fitheight[3];
    double heightB = k_height * (timeB - fitheight[2]) + fitheight[3];
    double heightC = k_height * (timeC - fitheight[2]) + fitheight[3];
    

    double timeCB = timeC - timeB;
    //std::cout << "heightA: " << heightA << " " << heightB << " " << heightC << " " << timeB << " " << timeCB << std::endl;

    if ((heightA <= heightB) && (heightB <= heightC))
    {
        double ttcTempA = 99.9;
        double ttcTempB = 99.9;
        double consC = 99.9;

        ttcTempA = (timeB) / ((heightB / heightA) -1. + alpha); //ms
        ttcTempB = (timeCB) / ((heightC / heightB) -1. + alpha);
        consC =1.0 + ((ttcTempB - ttcTempA)/(timeCB)); //??
        if (consC == 0.0)
        {
            consC = alpha;
        }
        //std::cout << "ttcTemp: " << ttcTempA << ttcTempA << consC << std::endl;

        double ttc1 = 99999.9;
        double ttc2 = 99999.9;
        ttc1 = ttcTempB * (1 - sqrt(1 + 2 * consC)) / consC; //ms
        ttc2 = ttcTempB * (1 + sqrt(1 + 2 * consC)) / consC;

        double ttcEmerg = 99999.9;
        if (ttc1 > 0.0 && ttc2 > 0.0) //两正表示在最小正数后就相撞
            ttcEmerg = (ttc1 > ttc2) ? ttc2 : ttc1;
        else if (ttc1 > 0.0 || ttc2 > 0.0) //一正一负代表在正的时间后相撞
            ttcEmerg = (ttc1 > 0.0) ? ttc1 : ttc2;
        else
            ttcEmerg = 99999.9; //两负数和两虚数代表不会相撞 
        

        //std::cout << "ttcEmerg: " << setiosflags(ios::fixed) << setprecision(3) << ttcEmerg / 1000.0 << "s" << " " << ttc1/1000.0 << " " << ttc2/1000.0 << std::endl;
        
        // 然后如果ttc是一个有效值，需要计算横向是否在范围内
        // 假设车长4.700m，车宽1.8m，前后各留1m空余，左右各留0.5m空余。所以，本车安全范围为长6.7m，宽2.8m。
        // 假设摄像头在车中间

        if (ttcEmerg <= 3000.0) 
        {        
            double xr4 = xr_sets[10];
            double xl4 = xl_sets[10];

            if (((xr4 * xl4) >= 0) && (fabs(xr4) > fabs(safe_disr)) && (fabs(xl4) > fabs(safe_disl)))
            {
                //不会碰撞，不用计算ttc
                //std::cout << "Horizon is not in the collision range ~ " << std::endl;
                objttc.ttc_existd = false;
                objttc.ttc_time = 88.8; //转化为秒
            }    
            else
            {
                /*至少有一边在安全距离范围内，则需要计算ttc*/
                std::vector<Point> point_setr;
                std::vector<Point> point_setl;
                cv::Point point_tempr;
                cv::Point point_templ;


                for (int i = 0; i != (TTCTRACKINGFRAMES); i++)
                {
                    if (xr_sets[i] != 0.0)
                    {   
                        point_tempr.x = time_interval * (frameid_sets[i] - frameid_sets[0]);
                        point_tempr.y = (xr_sets[i] ); //左侧距离为负 //按mm计算
                        point_setr.push_back(point_tempr);
                    }
                    else continue;
                    if (xl_sets[i] != 0.0)
                    {
                        point_templ.x = time_interval * (frameid_sets[i] - frameid_sets[0]);
                        point_templ.y = (xl_sets[i] );
                        point_setl.push_back(point_templ);
                    }
                    else continue;
                }

                //直线拟合,拟合方法采用最小二乘法或其他
                //拟合结果为一个四元素的容器，比如Vec4f - (vx, vy, x0, y0)
                //其中(vx, vy) 是直线的方向向量
                //(x0, y0) 是直线上的一个点
                std::vector<Point> point_sets [2] = {point_setr, point_setl};
                cv::Vec4f fitline;
                double preDis [2] = { }; //指定个数，默认初始化为0

                for (int points = 0; points != 2; points++)
                {
                    cv::fitLine(point_sets[points],fitline,CV_DIST_HUBER,0,0.01,0.01);                    
                    double k_line = fitline[1]/fitline[0];
                    double deltay = k_line*(ttcEmerg - fitline[2]) + fitline[3];//求出直线上 ttcEmergy 对应的点
                    preDis[points] = deltay;        
                }

                //std::cout << "preDis: " << preDis[0] <<" " << preDis[1] << std::endl;
                if ((fabs(preDis[0])+fabs(preDis[1]) != 0) && (preDis[0] * preDis[1]) <= 0) //正撞，摄像头左右都有
                {    
                    //肯定会撞，发出预警
                    //std::cout << "Predicted collision level is FIRST.Please take action!!" << std::endl;
                    objttc.ttc_existd = true;
                    objttc.ttc_time = ttcEmerg / 1000;
                    //std::cout << "ttcEmerg: " << setiosflags(ios::fixed) << setprecision(3) << ttcEmerg / 1000  << std::endl;
                }
                else if (preDis[0] > 0)
                {
                    if (preDis[0] > safe_disl)
                    {
                        //不会撞
                        //std::cout << "Not in collision path. The object moves to right" << std::endl; 
                        objttc.ttc_existd = false;
                        objttc.ttc_time = 88.8;
                    }    
                    else
                    {   
                        //肯定会撞，发出预警
                        //std::cout << "Predicted collision level is SECOND.Please take action!!" << std::endl; //在摄像头右前方撞
                        objttc.ttc_existd = true;
                        objttc.ttc_time = ttcEmerg / 1000;
                        //std::cout << "ttcEmerg: " << setiosflags(ios::fixed) << setprecision(3) << ttcEmerg / 1000  << std::endl;
                    }    
                }
                else if (preDis[0] < 0)// 预测距离为负
                {
                    if (fabs(preDis[1]) > fabs(safe_disr))
                    {
                        //不会撞
                        //std::cout << "Not in collision path.The object moves to left" << std::endl; //均在摄像头左边 撞
                        objttc.ttc_existd = false;
                        objttc.ttc_time = 88.8;
                    }   
                    else
                    {   
                        //肯定会撞，发出预警
                        //std::cout << "Predicted collision level is THIRD.Please take action!!" << std::endl;
                        objttc.ttc_existd = true;
                        objttc.ttc_time = ttcEmerg / 1000;
                        //std::cout << "ttcEmerg: " << setiosflags(ios::fixed) << setprecision(3) << ttcEmerg / 1000 << std::endl;
                    } 
                }
                point_setr.clear();
                point_setl.clear();
            }
        }
        else
        {
            //不会撞
            //std::cout << "Longitude is not in the collision range~" << std::endl << std::endl; 
            objttc.ttc_existd = false; 
            objttc.ttc_time = 88.8;
        }  
    }
    return objttc;
}

void TTC::TTCCalculate(objects &Tobj)
{   
    /* push new obj into the queue */
    this->_ped_queue.push_back(Tobj.PedObj);
    // this->_rider_queue.push_back(Tobj.RiderObj);
    this->_veh_queue.push_back(Tobj.VehObj);
    this->_road_queue.push_back(Tobj.RoadObj);

    // std::cout << "ttc pedobj " << std::endl;
    this->FetchDataFromQueue(this->_ped_queue, Tobj.PedObj);
    // std::cout << "ttc riderobj" << std::endl;
    // this->FetchDataFromQueue(this->_rider_queue, Tobj.RiderObj);
    // std::cout << "ttc vehobj " << std::endl;
    this->FetchDataFromQueue(this->_veh_queue, Tobj.VehObj);
    // std::cout << "ttc roadobj " << std::endl;
    this->FetchDataFromQueue(this->_road_queue, Tobj.RoadObj);
}