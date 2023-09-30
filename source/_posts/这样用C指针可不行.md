---
title: 这样用C指针可不行
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: c4010773
date: 2023-09-30 20:43:16
---

最近有人问了我一个C语言中使用指针时遇到的问题。下面是一个简化后的代码示例。

```c
#include <stdio.h>

int fix_var = 90;

void update_ptr(int *a) {
    int* temp = a;
    a = &fix_var;

    printf("adress of temp: %d \n", temp);
    printf("adress of a(in function): %d \n", a);
}

int main() {
    int *a = NULL;

    update_ptr(a);

    printf("adress of a: %d \n", a);
    return 0;
}
```
<!--more-->
编译运行的结果为：

```bash
adress of temp: 0
adress of a(in function): 4206608
adress of a: 0
```

他的疑问是，为什么我修改了函数内的指针a，结果在函数外面指针a却并没有变化。



这个其实是没有理解清楚变量的作用域。因为指针实际上也是一个变量。但我们常常把指针传入函数内部以便可以在函数内修改函数外面的某个变量值。从而被混淆地认为函数内也能改变传入的指针值。



给函数传入指针以修改函数外部变量的具体用法如下代码所示：

```c
#include <stdio.h>

// 交换两个整数的值
void swap(int *a, int *b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

int main() {
    int num1 = 10;
    int num2 = 20;

    printf("交换前的值：\n");
    printf("num1 = %d\n", num1);
    printf("num2 = %d\n", num2);

    // 调用swap函数来交换num1和num2的值
    swap(&num1, &num2);

    printf("交换后的值：\n");
    printf("num1 = %d\n", num1);
    printf("num2 = %d\n", num2);

    return 0;
}
```

编译执行的结果为：

```bash
交换前的值：
num1 = 10
num2 = 20
交换后的值：
num1 = 20
num2 = 10
```



实际上，我们通常不会考虑通过函数的形参去改变函数外部的指针。



## const与指针的缘分

我们通常将const和指针联合起来以达到防止修改指针指向的内容或者防止修改指针本身。

比如，我们将交换函数的形参都加上const。这个函数就编译不过了。

```c
// 交换两个整数的值
void swap(const int *a, const int *b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}
```

报错信息：

```bash
test.c: In function 'swap':
test.c:6:8: error: assignment of read-only location '*a'
     *a = *b;
        ^
test.c:7:8: error: assignment of read-only location '*b'
     *b = temp;
        ^
```

因为形参加了const后就表示函数内部不能修改形参指针所指向的内容了。而这里的交换函数是要修改指针指向的内容的，所以编译会出错。



这次我们把const加到星号后面。

```c
int b = 90;
void modify_ptr(int* const a) //这里只作演示
{
    a = &b;
}
```

编译报错信息：

```bash
test.c: In function 'modify_ptr':
test.c:14:7: error: assignment of read-only parameter 'a'
     a = &b;
       ^
```

注意对比之前的错误信息。这里的报错信息是说a（a为指针）是只读的，即a是不可修改的。之前的错误信息是说*a是只读的，即a指向的内容是不可修改的。



那如果两个const同时加呢？

```c
int b = 90;
void modify_ptr(const int* const a)
{
    *a = 10;
    a = &b;
}
```

编译的报错信息：

```bash
test.c: In function 'modify_ptr':
test.c:14:8: error: assignment of read-only location '*a'
     *a = 10;
        ^
test.c:15:7: error: assignment of read-only parameter 'a'
     a = &b;
       ^
```

两个const同时加说明指针和指针指向的内容都不能被修改。



希望这几个例子能帮助你理解。



更多C/C++的学习材料可以在公众号《**首飞**》内回复“机器人”关键字。我精心准备了C/C++，Python，Docker，Qt，ROS1/2等机器人行业常用技术资料，希望能帮你节省一些时间。





---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的机器人开发工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)





