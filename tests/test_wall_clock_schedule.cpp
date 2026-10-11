// Synthetic schedule only: no master, no sensor messages or state creation.
#include <ros/ros.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
int main(){
 ros::Time::init();ros::Time::setNow(ros::Time(10,0));std::atomic<bool> stop(false);std::atomic<unsigned> loops(0);
 std::thread worker([&]{while(!stop){++loops;ros::WallDuration(0.005).sleep();}});
 std::this_thread::sleep_for(std::chrono::milliseconds(250));unsigned frozen=loops;
 ros::Time::setNow(ros::Time(10,10000000));std::this_thread::sleep_for(std::chrono::milliseconds(100));unsigned resumed=loops-frozen;
 auto start=std::chrono::steady_clock::now();stop=true;worker.join();double exit_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 bool ok=frozen>=20&&frozen<=65&&resumed>=5&&resumed<=30&&exit_ms<100;
 std::cout<<"{\"frozen_loops_250ms\":"<<frozen<<",\"resumed_loops_100ms\":"<<resumed<<",\"exit_ms\":"<<exit_ms<<",\"passed\":"<<(ok?"true":"false")<<"}\n";return ok?0:1;
}
