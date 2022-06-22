#ifndef QUEUE_QUEUE_H
#define QUEUE_QUEUE_H
#include <cassert>
#include <thread_db.h>
#include <Eigen/Dense>

Eigen::Matrix4f gps_trans_array[3];
Eigen::Matrix4f vo_trans_array[3];
Eigen::Matrix4f odo_trans_array[3];


template<class T, int SIZE=2>
class Queue{
private:
    int front, rear, count;       //队头指针，队尾指针，元素个数
    T list[SIZE];                 //队列元素个数
public:
    Queue();                      //构造函数，初始化队头指针，队尾指针，元素个数
    void insert_1(const T &item);   //新元素入队
    T remove();                   //元素出队
    void clear();                 //清空队列
    const T &getFront() const;    //访问队首元素

    //测试队列状态
    int getLength() const;        //求队列长度（元素个数）
    bool isEmpty() const;         //判断队列是否为空
    bool isFull() const;          //判断队列是否已经满了
    void gps_show() const;
    void vo_show() const;
    void odo_show() const;
};

//构造函数，初始化队头指针,队尾指针，元素个数
template<class T,int SIZE>
Queue<T,SIZE>::Queue():front(0),rear(0),count(0){};

template<class T,int SIZE>
void Queue<T,SIZE>::insert_1(const T &item) {     //向队尾插入元素（入队）
    assert(count !=SIZE);
    count++;                                    //元素个数增1
    list[rear] = item;                          //向队尾插入元素
    rear = (rear+1) % SIZE;                     //队尾指针增1,用取余运算实现循环队列
}

template<class T,int SIZE>
void Queue<T,SIZE>::gps_show() const {
    int temp = front;
    for(int i = 0;i<count;i++)
    {
        // cout << list[temp]<<endl;
        gps_trans_array[i] = list[temp];
        if(temp == 2)
            temp = 0;
        else
            temp ++;
    }
}

template<class T,int SIZE>
void Queue<T,SIZE>::vo_show() const {
    int temp = front;
    for(int i = 0;i<count;i++)
    {
        // cout << list[temp]<<endl;
        vo_trans_array[i] = list[temp];
        if(temp == 2)
            temp = 0;
        else
            temp ++;
    }
}

template<class T,int SIZE>
void Queue<T,SIZE>::odo_show() const {
    int temp = front;
    for(int i = 0;i<count;i++)
    {
        // cout << list[temp]<<endl;
        odo_trans_array[i] = list[temp];
        if(temp == 2)
            temp = 0;
        else
            temp ++;
    }
}

template<class T, int SIZE>
T Queue<T,SIZE>::remove() {                    //删除队首元素，并返回该元素的值
    assert(count!=0);
    int temp=front;                            //记录下原先的队首指针
    count--;                                   //元素个数自减
    front=(front+1)%SIZE;                      //队首指针增1,取余以实现循环队列
    return list[temp];                         //返回首元素值
}

template<class T,int SIZE>
const T &Queue<T,SIZE>::getFront() const {     //访问队首元素（返回其值）
    return list[front];
}

template<class T,int SIZE>
int Queue<T,SIZE>::getLength() const {         //返回队列元素个数
    return count;
}

template<class T,int SIZE>
bool Queue<T,SIZE>::isEmpty() const {
    return count == 0;                          //测试队列空否
}

template<class T,int SIZE>
bool Queue<T,SIZE>::isFull() const{
    return count == SIZE;                       //测试队列满否
};

template<class T,int SIZE>
void Queue<T,SIZE>::clear(){                    //清空队列
    count = 0;
    front = 0;
    rear = 0;
};

#endif