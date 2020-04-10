#include"ObjectEstimation.h"

#define MAX_DISTANCE_MEASURE_X 150
#define MAX_DISTANCE_MEASURE_Y 30

#ifdef DISTANCEMEASURE
ObjectEstimation::ObjectEstimation(double fx, double fy , \
double cx, double cy, double k1, double k2, double p1, double p2, double k3, \
double pitch, double yaw, double roll, double Cam2Chamber, double Cam2Center ,double CamHeight):
    // clname({"Ped","Rider","Veh","Road"}),
    // clid({{0} , {1}, {2}, {3,4,5,6,7,8,9}}),
    clname({"Ped","Veh","Road"}),
    clid({{0} , {1}, {2,3,4,5,6,7,8}}),
    max_cosine_distance(0.9),
    nn_budget(10),
    maxid(256),
    groups(3),
    frameID(0),
    frame_num_saved(10),
    a_threshold(7.5), 
    r_threshold(0.65){

    this->trackers = new tracker*[clname.size()];
    for (unsigned int i = 0; i < clname.size(); i++)
        this->trackers[i] = new tracker(max_cosine_distance, nn_budget);

    this->IDDistribute = new IDDistributor(this->maxid, this->groups);

    this->dism.ConfigParam(fx, fy, cx, cy , \
            k1, k2, p1, p2, k3, pitch, yaw, roll, \
            Cam2Chamber, Cam2Center, CamHeight);
}
#else
ObjectEstimation::ObjectEstimation(const std::string &config_file):
    // clname({"Ped","Rider","Veh","Road"}),
    // clid({{0} , {1}, {2}, {3,4,5,6,7,8,9}}),
    clname({"Ped","Veh","Road"}),
    clid({{0} , {1}, {2,3,4,5,6,7,8}}),
    max_cosine_distance(0.9),
    nn_budget(10),
    maxid(256),
    groups(3),
    frameID(0),
    frame_num_saved(10),
    a_threshold(7.5), 
    r_threshold(0.65){

    cali.LoadCameraParametersFile(config_file);

    this->trackers = new tracker*[clname.size()];
    for (unsigned int i = 0; i < clname.size(); i++)
        this->trackers[i] = new tracker(max_cosine_distance, nn_budget);

    this->IDDistribute = new IDDistributor(this->maxid, this->groups);
}
#endif

ObjectEstimation::~ObjectEstimation(){
    for (unsigned int i = 0; i < clname.size(); i++)
    {
        delete this->trackers[i];
    }
    delete this->trackers;

    delete this->IDDistribute;
}

#ifdef USE_FPGA_PLANE
/**
 * @brief transform result from postprocess to tracking
 * @param dets - detections from network
 * @param width - frame width
 * @param height -frame height
*/
std::map<int,DETECTIONS> ObjectEstimation::getdetections(const std::vector<std::vector<float>>& dets, const int& width, const int& height, const int& frame_idx){
    this->frameID = frame_idx;
    map<int, DETECTIONS> vecdets;
    for (unsigned int j = 0; j < clname.size(); j++)
    {
        DETECTIONS ds;
        for (unsigned int i = 0; i < dets.size(); i++)
        {
            int cl = dets[i][4];
            auto it = find(clid[j].begin(), clid[j].end(), cl);
            if (it == clid[j].end()) continue;
           
            float x = dets[i][0];
            float y = dets[i][1];
            float w = dets[i][2];
            float h = dets[i][3];
            if(x < 0)
            {
                x = 0;
            }
            if(y < 0)
            {
                y = 0;
            }
            if((x + w) > width)
            {
                w = width-x;
            }
            if((y + h) > height)
            {
                h = height-y;
            }

            float conf = dets[i][5];

            #ifdef USE_FEATURE
            float* feature = new float[feature_dims];
            feature[0] = x / width;
            feature[1] = y / height;
            feature[2] = h / height;
            feature[3] = w / width;
            for (int j = 4; j < feature_dims; j++)
                feature[j] = dets[i][j+2];
            #endif 

            DETECTION_ROW tempRow;
            tempRow.cls = cl;
            tempRow.tlwh = DETECTBOX(x,y,w, h);
            tempRow.confidence = conf;
            #ifdef USE_FEATURE
            tempRow.feature = FEATURE(feature); // FEATURE(get_hog_feature(frame(cv::Rect(ltx, lty, w, h)), cv::Point2f(x, y))); //or nothing can setZero()
            #endif

            ds.push_back(tempRow);

            SaveDetections(this->frameID, ds);

            #ifdef USE_FEATURE
            if(feature)
            {
                delete feature;
                feature = nullptr;
            }
            #endif
        }  
        vecdets.insert(make_pair(j,ds));

    }
    return vecdets;
}
#else
std::map<int,DETECTIONS> ObjectEstimation::getdetections(const detection_with_class* dets,  const int& num, const int& width, const int& height, const int& frame_idx){
    this->frameID = frame_idx;
    map<int, DETECTIONS> vecdets;
    for (unsigned int j = 0; j < clname.size(); j++)
    {
        DETECTIONS ds;
        for (int i = 0; i < num; i++)
        {
            int cl = dets[i].best_class;
            auto it = find(clid[j].begin(), clid[j].end(), cl);
            if (it == clid[j].end()) continue;
            
            detection det = dets[i].det;
            
            float x = det.bbox.x * width;
            float y = det.bbox.y * height;
            float h = det.bbox.h * height;
            float w = det.bbox.w * width;
            float ltx = x - w/2.;
            float lty = y - h/2.;
            if(ltx < 0)
            {
                ltx = 0;
            }
            if(lty < 0)
            {
                lty = 0;
            }
            if((ltx + w) > width)
            {
                w = width-ltx;
            }
            if((lty + h) > height)
            {
                h = height-lty;
            }

            float conf = 0.f;

            float* feature = new float[feature_dims];
            feature[0] = x;
            feature[1] = y;
            feature[2] = h;
            feature[3] = w;
            for (int j = 4; j < feature_dims; j++){
                feature[j] = det.prob[j-4];
                if(conf < feature[j]){
                    conf = feature[j];
                }
            }

            DETECTION_ROW tempRow;
            tempRow.cls = cl;
            tempRow.tlwh = DETECTBOX(ltx,lty,w, h);
            tempRow.confidence = conf;
            #ifdef USE_FEATURE
            tempRow.feature = FEATURE(feature); // FEATURE(get_hog_feature(frame(cv::Rect(ltx, lty, w, h)), cv::Point2f(x, y)));//or nothing can setZero()
            #endif

            ds.push_back(tempRow);

            SaveDetections(this->frameID, ds);

            if(feature)
            {
                delete feature;
                feature = nullptr;
            }
        }  
        vecdets.insert(make_pair(j,ds));
        
    }
    return vecdets;
}
#endif

objects ObjectEstimation::UpdateAll(map<int, DETECTIONS> d, const std::map<int, Point2f>& W,const cv::Point2f CamVelocity){
    objects objs;
    this->camvelocity = CamVelocity;

    for (unsigned int i = 0; i < clname.size(); i++ ){
        switch (i)
        {
        case 0:
            //cout << "ped size: "<< d[i].size() << endl;
            objs.PedObj = this->UpdateOneClass<object>(d[i], this->trackers[i], this->objects_main.PedObj, 0, object::TransferType::WidthHeight);
            break;
        case 1:
        //     //cout << "rider size: "<< d[i].size() << endl;
        //     objs.RiderObj = this->UpdateOneClass<object>(d[i], this->trackers[i],  this->objects_main.RiderObj, 1, object::TransferType::WidthHeight);
        //     break;
        // case 2:
            //cout << "veh size: "<< d[i].size() << endl;
            objs.VehObj = this->UpdateOneClass<object>(d[i], this->trackers[i],  this->objects_main.VehObj, 1, object::TransferType::WidthHeight);
            break;
        case 2:
            //cout << "roadobj size: "<< d[i].size() << endl;
            objs.RoadObj = this->UpdateOneClass<object>(d[i], this->trackers[i],  this->objects_main.RoadObj, 2, object::TransferType::WidthLength);
            break;        
        default:
            break;
        }     
    }

    this->objects_main.PedObj.clear();
    // this->objects_main.RiderObj.clear();
    this->objects_main.VehObj.clear();
    this->objects_main.RoadObj.clear();
    this->objects_main = objs;

    this->ttc.TTCCalculate(this->objects_main);
    this->SaveObjects(this->objects_main, W);
    return this->objects_main;
}

template<typename Tobj>
map<int, Tobj> ObjectEstimation::UpdateOneClass(const DETECTIONS detections, tracker* tr, map<int,Tobj> obj_old,  int index, object::TransferType worldtype){   
    map<int, Tobj> obj_with_trkID;
    tr->predict(); // 对上一帧所有的trk进行预测
    tr->update(detections, this->IDDistribute, index, this->cali, worldtype);

    for(Track& track : tr->tracks) {
        if(!track.is_confirmed() || track.time_since_update() > 1) continue;
        Tobj obj = track;
        obj.frame_index_update(this->frameID);
        obj_with_trkID.insert(make_pair(track.track_id(), obj));
    };

    return obj_with_trkID;
}

void ObjectEstimation::SaveDetections(const int& frame_idx, const DETECTIONS& dets){
    if(dets_with_frameID.find(frame_idx) == dets_with_frameID.end()) {
        if(dets_with_frameID.size() < frame_num_saved){
            dets_with_frameID.insert(make_pair(frame_idx, dets));
        }else{
            dets_with_frameID.erase(dets_with_frameID.begin());
            dets_with_frameID.insert(make_pair(frame_idx, dets));
        }
    } else {
        dets_with_frameID[frame_idx].push_back(dets[dets.size()-1]);
    }
}

void ObjectEstimation::SaveObjects(objects& objs, const std::map<int, Point2f> &W){
    if(QueueObject.size() < frame_num_saved){
        QueueObject.push_back(objs);
    }else{
        QueueObject.erase(QueueObject.begin());
        QueueObject.push_back(objs);  
    }

    std::vector<int> IDlist;
    auto objs_curr = *(QueueObject.end()-1);
    auto veh_curr = objs_curr.VehObj;
    for (auto it = veh_curr.begin(); it != veh_curr.end(); ++it)
        IDlist.push_back(it->first);
    
    for (auto it = QueueObject.begin(); it < QueueObject.end(); it++)
    {
        for (auto id = IDlist.begin(); id!=IDlist.end(); id++)
        {
            // find the veh_obj in the previous frames
            auto veh_obj = it->VehObj.find(*id);
            if(veh_obj == it->VehObj.end()) continue;
            cv::Point2f veh_obj_pos = cv::Point2f(veh_obj->second.worldposition().x, veh_obj->second.worldposition().y);

            // find trajectory in the previous frames
            int frame_index = veh_obj->second.frame_index();
            auto it_self = W.find(frame_index);
            if(it_self == W.end()) continue;
            cv::Point2f veh_ego_pos = it_self->second;

            cv::Point2f dif;
            dif.x = veh_obj_pos.x + veh_ego_pos.x;
            float distance = sqrt(dif.dot(dif));
            
            objs.VehObj[*id].Dislist.insert(std::make_pair(frame_index, distance));
            // cout << veh_obj_pos << " " << veh_ego_pos << " " << distance << endl;
        }
        //cout << endl;
    }
    EstimateMoveState(objs);
}

void ObjectEstimation::EstimateMoveState(objects& objs_){
    for (auto it = objs_.VehObj.begin(); it != objs_.VehObj.end(); ++it)
    {
        #ifdef DEBUG_INFO
        cout << "VehObj ID: " << it->first <<endl;
        #endif
        auto dis_list = it->second.Dislist;

        unsigned int num = dis_list.size();
        if(num > 2){
            auto iter0 = dis_list.begin();
            int frame_id_curr = iter0->first;
            int frame_id_next = (++iter0)->first;
            float time_interval = static_cast<float>((frame_id_next - frame_id_curr))/FRAMEFREQUENCY;

            int cnt = 0;
            float sumXX = 0.0, sumX = 0.0, sumXY = 0.0, sumY = 0.0;
            for (auto iter = dis_list.begin(); iter != dis_list.end(); ++iter){
                #ifdef DEBUG_INFO
                cout << "frame id: " << iter->first << "    distance: " << iter->second << endl;
                #endif
                float dt = time_interval * cnt++;
                sumXX += dt * dt;
                sumX += dt;
                sumXY += dt * iter->second;
                sumY += iter->second;
            }

            float a, b, tmp = 0;
            if(tmp = (num * sumXX - sumX * sumX)){
                a = (num * sumXY - sumX * sumY) / tmp;
                b = (sumXX * sumY - sumX * sumXY) / tmp;
            }
            else{
                a = 1;
                b = 0;
            }

            float meanX = sumX / num;
            float meanY = sumY /num;

            cnt = 0;
            float varX = 0.0, varY = 0.0, covXY = 0.0;
            for(auto iter = dis_list.begin(); iter != dis_list.end(); ++iter){
                float dt = time_interval * cnt++;
                varX += (dt - meanX) * (dt -meanX);
                varY += (iter->second - meanY) * (iter->second - meanY);
                covXY += (dt -meanX) * (iter->second - meanY);
            }
            float r = covXY / (sqrt(varX) * sqrt(varY));

            #ifdef DEBUG_INFO
            std::cout << "a=" << a << " b=" << b << " r=" << r << endl;
            std::cout << "a_threshold=" << a_threshold << " r_threshold=" << r_threshold << endl;
            #endif

            if(a < -a_threshold && r < -r_threshold){
                it->second.set_movestate(MoveState::MoveClosing);
            }
            else if(a > a_threshold && r > r_threshold){
                it->second.set_movestate(MoveState::MoveFarAway);
            }
            else{
                it->second.set_movestate(MoveState::NotMove);
            }

        }

        #ifdef DEBUG_INFO
        cout << "state: " << it->second.get_movestate() << " {0: NotMove, 1: MoveClosing, 2: MoveFarAway, 3: Unknown}" << endl;
        #endif
    }
}

bool ObjectEstimation::getFrameDetections(int frame_idx, DETECTIONS& res)
{
    if(dets_with_frameID.empty()){
	    return false;
    }
    
    res = dets_with_frameID[frame_idx];
    
    return true;
}

bool ObjectEstimation::getFrameTracks(int frame_idx, objects& objs)
{
    if(QueueObject.empty()){
        return false;
    }

    for(size_t i=0; i<QueueObject.size(); ++i){
        PEDOBJ ped_obj = QueueObject[i].PedObj;
        for(size_t p=0; p<ped_obj.size(); ++p){
            if(frame_idx == ped_obj[p].frame_index()){
                objs.PedObj = ped_obj;
            }
        }

        VEHOBJ veh_obj = QueueObject[i].VehObj;
        for(size_t v=0; v<veh_obj.size(); ++v){
            if(frame_idx == veh_obj[v].frame_index()){
                objs.VehObj = veh_obj;
            }
        }
        
        ROADOBJ road_obj = QueueObject[i].RoadObj;
        for(size_t r=0; r<road_obj.size(); ++r){
            if(frame_idx == road_obj[r].frame_index()){
                objs.RoadObj = road_obj;
            }
        }
    }
    
    return true;
}


void ObjectEstimation::drawdetect(Mat &image, map<int, DETECTIONS> d, const std::map<int, cv::Point2f> W, const objects objs){

    // DETECTIONS frame_dets;
    // Obj_Esti.getFrameDetections(step, frame_dets);

    // for(size_t i=0; i<frame_dets.size(); ++i){
    //     DETECTBOX tmpbox = frame_dets[i].tlwh;
    //     cv::rectangle(frame, cv::Rect(tmpbox[0], tmpbox[1], tmpbox[2], tmpbox[3]), cv::Scalar(0, 0, 255), 2);
    // }

    // objects frame_objs;
    // Obj_Esti.getFrameTracks(step, frame_objs);

    // PEDOBJ frame_ped = frame_objs.PedObj;
    // VEHOBJ frame_veh = frame_objs.VehObj;
    // ROADOBJ frame_road = frame_objs.RoadObj;
    // for(size_t i=0; i<frame_ped.size(); ++i){
    //     cv::rectangle(frame, cv::Rect(frame_ped[i].box()), cv::Scalar(255, 255, 0));
    // }
    // for(size_t i=0; i<frame_veh.size(); ++i){
    //     cv::rectangle(frame, cv::Rect(frame_veh[i].box()), cv::Scalar(255, 255, 0));
    // }
    // for(size_t i=0; i<frame_road.size(); ++i){
    //     cv::rectangle(frame, cv::Rect(frame_road[i].box()), cv::Scalar(255, 255, 0));
    // }

    Mat imgworld = Mat::zeros(600,400, CV_8UC3);
    Mat imgego = Mat::zeros(400,400, CV_8UC3);

    for(unsigned int i = 0; i < d.size(); i++){
        DETECTIONS ds = d[i];
        for(unsigned int k = 0; k < ds.size(); k++){
            DETECTBOX tmpbox = ds[k].tlwh;
            cv::Rect rect(tmpbox(0), tmpbox(1), tmpbox(2), tmpbox(3));
            cv::rectangle(image, rect, cv::Scalar(0, 0, 255), 4);
            // cv::putText(image, std::to_string(ds[i].cls), cv::Point(rect.x, rect.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2);
        }
    }
   

    this->draw_image(objs, image);
    this->draw_ground(objs, imgworld);
    this->draw_ego(W, imgego);
    // this->draw_Calibrater(image, this->cali); // draw lane_lines_filtered and m_vanish_point_of_lanes

    cv::Mat img = image_add(image, imgworld, imgego, 1);

    #ifdef USE_FPGA_PLANE
    #else
    cv::imshow("frame", img);
    cv::waitKey(1);
    #endif
    
    // char image_name[100];  //保存路径
    // sprintf(image_name, "%s%d%s", "test/images/results", this->frame_id, ".jpg");   //指定保存路径
    // imwrite(image_name, image);  //保存图像
}

void ObjectEstimation::draw_image(const objects &objs, Mat& image){
    PEDOBJ ped = objs.PedObj;
    // RIDEROBJ rider = objs.RiderObj;
    VEHOBJ veh = objs.VehObj;
    ROADOBJ road = objs.RoadObj;
    this->draw_image_per_class<PEDOBJ>(image, ped, clname[0]);
    // this->draw_image_per_class<RIDEROBJ>(image, rider, clname[1]);
    this->draw_image_per_class<VEHOBJ>(image, veh, clname[1]);
    this->draw_image_per_class<ROADOBJ>(image, road, clname[2]);
}

void ObjectEstimation::draw_ground(const objects &objs, Mat& image){

    for (auto item = QueueObject.begin(); item != QueueObject.end(); item++){
        PEDOBJ ped = item->PedObj;
        // RIDEROBJ rider = item->RiderObj;
        VEHOBJ veh = item->VehObj;
        ROADOBJ road = item->RoadObj;
        if(item == (QueueObject.end()-1)){
            this->draw_ground_per_class<PEDOBJ>(image, ped, Scalar(0,255,255), clname[0]);
            // this->draw_ground_per_class<RIDEROBJ>(image, rider, Scalar(0,0,255), clname[1]);
            this->draw_ground_per_class<VEHOBJ>(image, veh, Scalar(0,0,255), clname[1]);
            this->draw_ground_per_class<ROADOBJ>(image, road, Scalar(0,0,255), clname[2]);
        }else
        {
            this->draw_ground_per_class<PEDOBJ>(image, ped, Scalar(255,255,0), clname[0]);
            // this->draw_ground_per_class<RIDEROBJ>(image, rider, Scalar(255,255,0), clname[1]);
            this->draw_ground_per_class<VEHOBJ>(image, veh, Scalar(255,255,0), clname[1]);
            this->draw_ground_per_class<ROADOBJ>(image, road, Scalar(255,255,0), clname[2]);
        }

    }
    cv::line(image,Point(image.cols/2.+this->camvelocity.y *20,image.rows-this->camvelocity.x*20),
    Point(image.cols/2.,image.rows),Scalar(0,0,255),2);

    float leftlane_x, left_leftlane_x, rightlane_x, right_rightlane_x;
    float leftlane_y, left_leftlane_y, rightlane_y, right_rightlane_y;
    leftlane_y = 1.7 / MAX_DISTANCE_MEASURE_Y *image.cols + image.cols / 2;
    left_leftlane_y = 5.2 / MAX_DISTANCE_MEASURE_Y*image.cols+ image.cols / 2;
    rightlane_y = -1.7 / MAX_DISTANCE_MEASURE_Y*image.cols+ image.cols / 2;
    right_rightlane_y = -5.2 / MAX_DISTANCE_MEASURE_Y*image.cols+ image.cols / 2;

    leftlane_x = image.rows - 30. / MAX_DISTANCE_MEASURE_X*image.rows;
    left_leftlane_x = image.rows - 30. / MAX_DISTANCE_MEASURE_X*image.rows;
    rightlane_x = image.rows - 30. / MAX_DISTANCE_MEASURE_X*image.rows;
    right_rightlane_x = image.rows - 30. / MAX_DISTANCE_MEASURE_X*image.rows;

    cv::line(image,Point(leftlane_y,leftlane_x),Point(leftlane_y,image.rows),Scalar(0,0,255),2);
    cv::line(image,Point(left_leftlane_y,left_leftlane_x),Point(left_leftlane_y,image.rows),Scalar(0,0,255),2);
    cv::line(image,Point(rightlane_y,rightlane_x),Point(rightlane_y,image.rows),Scalar(0,255,0),2);
    cv::line(image,Point(right_rightlane_y,right_rightlane_x),Point(right_rightlane_y,image.rows),Scalar(0,255,0),2);

    // cv::line(frame_world, Point(world.y*10.+150.,600-world.x*10.),
    // Point(world.y*10.+150.+vel.y*10.,600-world.x*10.+vel.x*10.), cv::Scalar(255,0,0),2, 8, 0);

}

void ObjectEstimation::draw_ego(const std::map<int, Point2f>& W, cv::Mat &imgego){
	for (auto it = W.begin(); it != W.end(); it++) {
		cv::Point2f WorldP ;
		WorldP.y = -it->second.x*5;
		WorldP.x = it->second.y*5 + imgego.cols/2.;	
		circle(imgego, WorldP, 2,255,2);
	}
	//imshow("test", imgego);
	//waitKey(30);
}

void ObjectEstimation::draw_Calibrater(cv::Mat &img, const HRYTCalibrater &cali){
    cv::Point2f p1,p2,pw;

    for(int i=10; i<=200;i+=20){
        pw.x = i;
        pw.y = -2.;
        cali.HRYTGroundPointToPixelPointOffline(p1, pw);	
        cv::circle(img, p1, 2, 255, 2);
        pw.x = i;
        pw.y = 2.;
        cali.HRYTGroundPointToPixelPointOffline(p2, pw);	
        cv::circle(img, p2, 2, 255, 2);

        cv::line(img,p1,p2,Scalar(0,0,255),2);

    }
}

template<typename Tobj>
void ObjectEstimation::draw_image_per_class(Mat &image, const Tobj obj, const std::string name){
    for(auto k = obj.begin(); k != obj.end(); k++)
    {
        cv::Rect2f rect = k->second.box();
        cv::Rect2f world =k->second.worldposition();
        cv::Point2f vel = k->second.velocity();
        unsigned int cls = k->second.objectclass();
        unsigned int id = k->first;
        ObjectState objst = k->second.objstate;
        MoveState mvst = k->second.get_movestate();
        stringstream ss, label, absttc;

        #ifdef DEBUG_INFO
        cout << "Class : "<< k->second.objectclass() << endl;
        cout << "Pixel : " << rect << ", World : " << world << endl;
        cout << "Velocity : " << vel ;
        cout << "TTC existed :" << k->second.ttc().ttc_existd << ", TTC time : " << k->second.ttc().ttc_time << endl;
        cout << "Pixel Velocity : " << k->second.vel_pix << " Closing state : " << k->second.objstate <<" Moving state : " << k->second.get_movestate() << endl;
        #endif

        cv::Point2f tl = k->second.topleft();
        cv::Point2f br = k->second.leftbottom();

        // print obj info class/track id/objstate/mvstate AND distance
        cv::rectangle(image, rect, cv::Scalar(255, 255, 0));
        cv::putText(image, std::to_string(cls), cv::Point(tl.x, tl.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);
        cv::putText(image, "/"+std::to_string(id), cv::Point(tl.x+10, tl.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 1);
        cv::putText(image, "/"+std::to_string(objst), cv::Point(tl.x+10*3, tl.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1);
        cv::putText(image, "/"+std::to_string(mvst), cv::Point(tl.x+10*5, tl.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 0), 1);
        ss << setprecision(5) << world.x ;// << setprecision(3) << world.y << setprecision(3) << world.width << setprecision(3) << world.height << "Pixel : " << k->second.vel_pix << endl;
        cv::putText(image, ss.str(), cv::Point(br.x, br.y+20), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
    }
}

template<typename Tobj>
void ObjectEstimation::draw_ground_per_class(Mat& image, const Tobj obj, cv::Scalar scalar, std::string name){
    for (auto it = obj.begin(); it != obj.end(); it++)
    {
        //cv::Point2f vel = k->second.velocity();

        Rect2f world = it->second.worldposition();
        float y = image.rows - world.x / MAX_DISTANCE_MEASURE_X * image.rows;
        float x = -world.y / MAX_DISTANCE_MEASURE_Y * image.cols + image.cols / 2.;
        cv::circle(image,Point(x,y),2, scalar,2);

        Point2f  vel = it->second.velocity();// + Point2f(this->camvelocity.x,this->camvelocity.y);

        cv::line(image, cv::Point(x,y), cv::Point(x+vel.y*10, y+vel.x*10), Scalar(255,0,0), 2, 8);   
    }
}
