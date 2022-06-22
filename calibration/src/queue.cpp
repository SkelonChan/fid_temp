#include <iostream>
#include <ros/ros.h>
#include<queue>
#include <Eigen/Dense>
#include <std_msgs/Int16.h>
using namespace std;
 

 
 
// int main() {
//     int tmp = 0;
//     int length = 0;

//     Queue<int> *aqueue = new Queue<int>();
//     cout<<"main"<<endl;

//     //插入元素
//     aqueue->insert(10);
//     aqueue->insert(20);
//     aqueue->insert(30);

//     //访问队首元素
//     tmp = aqueue->getFront();
//     cout << "tmp=" << tmp <<endl;

//     //元素出队
//     aqueue->remove();

//     //获取队列长度
//     length = aqueue->getLength();
//     cout << "length=" << length << endl;

//     //判断队列空否
//     cout << "队列是否为空：" << aqueue->isEmpty() << endl;

//     //判断队列满否
//     cout << "队列满否（总长度为50）:" << aqueue->isFull() << endl;

// }

Queue<Eigen::Matrix4f> *aqueue = new Queue<Eigen::Matrix4f>();

class queue2
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub1;
    // ros::Subscriber sub2;
    ros::Publisher pub1;
public:
    queue2();
    ~queue2();
    void sub_cb(const std_msgs::Int16::ConstPtr &p);
};

queue2::queue2()
{
    sub1 = nh.subscribe<std_msgs::Int16>("eigen",2,&queue2::sub_cb,this);
    pub1 = nh.advertise<Eigen::Matrix4f>("/pub",3);
}

queue2::~queue2()
{
}

void queue2::sub_cb(const std_msgs::Int16::ConstPtr &p)
{
    if(aqueue->isFull())
        aqueue->remove();
    aqueue->insert(p->data * Eigen::Matrix4f::Identity());
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "temp");
    // queue.;
    
    queue2 ttt;
    ros::spin();
    return 0;
}
