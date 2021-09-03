#pragma once
#include "../tool/tgaimage.h"
#include "../tool/geometry.h"

//首先这毫无疑问是一个shader的基类头文件，那下面定义在结构体外的函数还有变量是什么东西呢。可以看做是全局的一些变量和函数，会在shader文件中使用，并且在
//包含此shader头文件的地方也可以直接使用

extern Matrix ModelView;//这两个矩阵因为是全局的，所以可以直接在下面的函数体中被赋值
extern Matrix Projection;
//窗口的宽高，在triangle实现里会被用到
extern const int WINDOW_HEIGHT;
extern const int WINDOW_WIDTH;


void viewport(int x, int y, int w, int h); //ModelView矩阵
void projection(float fovy, float aspect, float near, float far);//投影矩阵 
void lookat(Vec3f eye, Vec3f center, Vec3f up); //视口变换矩阵
void set_color(unsigned char* framebuffer, int x, int y, TGAColor c);//设置framebuffer某一点颜色值，会在triangle中调用

void clip_prepared(int iface, int nthvert, Model* model);
int clipping();

//这就是IShader基类
struct IShader {
    mat<2, 3, float> varying_uv;  // 三角形的纹理坐标，由顶点着色器赋值，并作为片元着色器的输入
    mat<3, 3, float> world_vert; //三角形顶点的世界坐标
    mat<4, 3, float> varying_tri; // 三角形的三个顶点，这里的顶点是在裁剪空间里的
    mat<3, 3, float> varying_nrm; // 三个顶点的法向量，这里存的法向量已经是校正过的
    mat<3, 3, float> ndc_tri;     // 这里三角形的顶点已经在标准正方体视见体内
    virtual ~IShader();
    virtual Vec4f vertex(int index, int vert) = 0;//纯虚函数，这样设计是合理的，因为所有的shader都必须实现顶点着色器和片源着色器
    virtual bool fragment(Vec3f bar, TGAColor &color, Model* model) = 0;
};

struct PhongShader : public IShader {

   
    virtual Vec4f vertex(int index, int vert);
    virtual bool fragment(Vec3f bar, TGAColor& color, Model* mdoel);
};

void triangle(mat<4, 3, float>& clipc, IShader& shader, unsigned char* framebuffer, float* zbuffer, Model* model);


