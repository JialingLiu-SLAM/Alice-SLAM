//
//  ViewController+MetalRendering.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/8.
//  Copyright © 2023 栗大人. All rights reserved.
//



#import "ViewController+MetalRendering.h"
#import "OBJModel.h"
#import "Shared.h"
#import "Transforms.h"

using namespace cv;


@implementation ViewController (MetalRendering)

    Renderer *_renderer;
    id<MTLBuffer> _vertexBuffer;
    id<MTLBuffer> _indexBuffer;
    id<MTLBuffer> _uniformBuffer;
    simd::float4x4 _projectionMatrix;


static float DegToRad(float deg)
{
    return deg * (M_PI / 180);
}
     float aspect ;
- (void)initRendering {
//    _renderer = [[Renderer alloc] initWithLayer:(CAMetalLayer *)self.metalView.layer];
    _renderer = [[Renderer alloc] initWithLayer:(CAMetalLayer *)self.metalARView.layer];

    _renderer.vertexFunctionName = @"vertex_main";
    _renderer.fragmentFunctionName = @"fragment_main";
    
      aspect = self.metalARView.bounds.size.width / self.metalARView.bounds.size.height;
    
    cout<<"物体宽高="<<self.metalARView.bounds.size.width<<" , "<<self.metalARView.bounds.size.height<<endl;
    [self loadModel];
}

- (void)loadModel {
    NSURL *modelURL = [[NSBundle mainBundle] URLForResource:@"spongebob" withExtension:@"obj"];
    OBJModel *teapot = [[OBJModel alloc] initWithContentsOfURL:modelURL];
    
    std::cout<<"物体的数量"<<[teapot groupCount]<<std::endl;
    OBJGroup *baseGroup = [teapot groupAtIndex:1];
    if (baseGroup)
    {
        _vertexBuffer = [_renderer newBufferWithBytes:baseGroup->vertices
                                                       length:sizeof(Vertex) * baseGroup->vertexCount];
        _indexBuffer = [_renderer newBufferWithBytes:baseGroup->indices
                                                      length:sizeof(IndexType) * baseGroup->indexCount];
        
        std::cout<<"进来了"<<std::endl;
    }
}

- (void)drawObjectWith:(const Mat&)R andT:(const Mat&)T andisTracking:(bool)isTracking{
    //将旋转矩阵转化为四元数
    float qx=0.0,qy=0.0,qz=0.0,qw=0.0;
    float t0=0.0,t1=0.0,t2=10.0;
    
    if(isTracking==true){
        qw = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
        qx = (R.at<float>(2,1) - R.at<float>(1,2)) / (4*qw) ;
        qy = -(R.at<float>(0,2) - R.at<float>(2,0)) / (4*qw) ;
        qz = -(R.at<float>(1,0) - R.at<float>(0,1)) / (4*qw) ;
        t0 = -T.at<float>(0);
        t1 = T.at<float>(1);
        t2 = -T.at<float>(2);
    }
    
    
    //Mat T_p = R.t()*T;
    
    //将四元数转化为旋转矩阵，即r1 r2 r3. 并且将平移矩阵填充到r4
    //simd::float4 r1 = { 1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, 0 };
    //simd::/Users/zhangjianhua/Desktop/VIO/VIO/Categoriesfloat4 r2 = { 2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, 0 };
    //simd::float4 r3 = { 2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0 };
    simd::float4 r1 = { 1 - 2*qy*qy - 2*qz*qz, 2*qx*qy + 2*qz*qw, 2*qx*qz - 2*qy*qw, 0 };
    simd::float4 r2 = { 2*qx*qy - 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz + 2*qx*qw, 0 };
    simd::float4 r3 = { 2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0 };
    simd::float4 r4 = { t0, t1, t2, 1 };
    
    //simd::float4 r1 = { R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), 0 };
    //simd::float4 r2 = { R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), 0 };
    //simd::float4 r3 = { R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), 0 };
    //simd::float4 r4 = { -T.at<float>(0), T.at<float>(1), T.at<float>(2), 1 };
    
    simd::float4x4 poseMatrix = {r1, r2, r3, r4};
    simd::float4x4 modelMatrix = poseMatrix*matrix_uniform_scale(0.3);//位姿缩放比例0.3
    
    simd::float4x4 viewMatrix = Identity();
    //viewMatrix.columns[3].z = -0.5; // translate camera back along Z axis
    //投影矩阵 宽高比，视野角，近平面距离，远平面距离
    const float near = 0.02;//0.02
    const float far = 100;//100
//   const float aspect = self.metalView.bounds.size.width / self.metalView.bounds.size.height;
//    const float aspect = self.ARview.bounds.size.width / self.ARview.bounds.size.height;

   // cout<<"aspect="<<aspect<<endl;
    simd::float4x4 projectionMatrix = PerspectiveProjection(aspect, DegToRad(75), near, far);//75
    
    
    
    Uniforms uniforms;
     //模型变换矩阵
    simd::float4x4 modelView =  viewMatrix*modelMatrix;
    uniforms.modelViewMatrix = modelView;
    
    simd::float4x4 modelViewProj = projectionMatrix * modelView;
    uniforms.modelViewProjectionMatrix = modelViewProj;
    
    simd::float3x3 normalMatrix = { modelView.columns[0].xyz, modelView.columns[1].xyz, modelView.columns[2].xyz };
    uniforms.normalMatrix = simd::transpose(simd::inverse(normalMatrix));
    
    _uniformBuffer = [_renderer newBufferWithBytes:(void *)&uniforms length:sizeof(Uniforms)];
    
    [self redraw];
}

- (void)drawObjectWith2:(const Mat&)R andT:(const Mat&)T {
    
    
    
    float t0 = 0.0,t1 = 0.0,t2 = 10.0;
//    150 , 230
   
//    这样y轴为负 物体是正的
//        qw = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
//        qy = -(R.at<float>(2,1) - R.at<float>(1,2)) / (4*qw) ;
//        qx = -(R.at<float>(0,2) - R.at<float>(2,0)) / (4*qw) ;
//        qz = -(R.at<float>(1,0) - R.at<float>(0,1)) / (4*qw) ;
//        t1 = T.at<float>(0) ;//* 6.7
//        t0 = T.at<float>(1);
//        t2 = -T.at<float>(2);
    
//    qw = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
//    qx = -(R.at<float>(2,1) - R.at<float>(1,2)) / (4*qw) ;
//    qy = -(R.at<float>(0,2) - R.at<float>(2,0)) / (4*qw) ;
//    qz = -(R.at<float>(1,0) - R.at<float>(0,1)) / (4*qw) ;
    t0 = T.at<float>(0) ;//* 6.7
    t1 = T.at<float>(1);
    t2 = T.at<float>(2);
    
    
//    simd::float4 r1 = { 1 - 2*qy*qy - 2*qz*qz, 2*qx*qy + 2*qz*qw, 2*qx*qz - 2*qy*qw, 0 };
//    simd::float4 r2 = { 2*qx*qy - 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz + 2*qx*qw, 0 };
//    simd::float4 r3 = { 2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0 };
//    simd::float4 r4 = { t0, t1, t2, 1 };
    
    simd::float4 r1 = {R.at<float>(0,0), R.at<float>(1,0), R.at<float>(2,0), 0 };
    simd::float4 r2 = { R.at<float>(0,1), R.at<float>(1,1),R.at<float>(2,1), 0 };
    simd::float4 r3 = {R.at<float>(0,2),R.at<float>(1,2), R.at<float>(2,2), 0 };
    simd::float4 r4 = { t0, t1, t2, 1 };
    
    simd::float4x4 viewMatrix;
    
    
        viewMatrix = {r1, r2, r3, r4};
    
    simd::float4x4 viewMatrix_ljl;
    simd::float4 ljl1 = { 0, 1, 0, 0 };
    simd::float4 ljl2 = { 1,0, 0, 0 };
    simd::float4 ljl3 = { 0, 0, -1, 0 };
    simd::float4 ljl4 = { 0, 0, 0, 1 };
    viewMatrix_ljl = {ljl1, ljl2, ljl3, ljl4};
    
    
    simd::float3 ygx = { 1.0, 0.0, 0.0 }; //change this for placing model
    simd::float4x4 modelMatrix = Rotation(ygx, DegToRad(0)) * matrix_uniform_scale(0.1);
    
    int ratio = 1;
//    modelMatrix.columns[3][0] = pos[0]*ratio;
//    modelMatrix.columns[3][1] = -pos[1]*ratio;
//    modelMatrix.columns[3][2] = pos[2]*ratio;
        modelMatrix.columns[3][0] = 0;
        modelMatrix.columns[3][1] =0;
        modelMatrix.columns[3][2] =0;
    
//    float f_x =458.654;
//    float f_y = 457.296;
//    float c_x = 367.215 ;
//    float c_y = 248.375 ;
//    float width = 230 ;//752.0 499.5
//    float height = 150 ;//480.0 374.5
    float f_x = 530.233f;
    float f_y = 531.082f;
    float c_x = 250.286f;
    float c_y = 316.520f;
    float width = 480.0;
    float height = 640.0 ;
    float far_plane = 100000.0;
    float near_plane = 0.01;
    
    float f = 1.0 / tan(f_x / 2.0);
//    simd::float4 p1 = { f/aspect, 0.0f, 0.0f, 0.0f };
//    simd::float4 p2 = { 0.0f, f, 0.0f, 0.0f };
//    simd::float4 p3 = { 0.0f,0.0f, far_plane/(far_plane - near_plane), 1.0f };
//    simd::float4 p4 = { 0.0f, 0.0f, -far_plane*near_plane/(far_plane - near_plane), 0.0f };
    
    simd::float4 p1 = { 2*f_x/width, 0.0f, 0.0f, 0.0f };
    simd::float4 p2 = { 0.0f, 2*f_y/height, 0.0f, 0.0f };
    simd::float4 p3 = { 1.0f - 2*c_x/width, 2*c_y/height - 1.0f, -(far_plane + near_plane)/(far_plane - near_plane), -1.0f };
    simd::float4 p4 = { 0.0f, 0.0f, -2.0f*far_plane*near_plane/(far_plane - near_plane), 0.0f };
    simd::float4x4 projectionMatrix = {p1, p2, p3, p4};
    

//    float _fov = 65.0f * (M_PI / 180.0f);
//    float _nearPlane = 1.0f;
//    float _farPlane = 1500.0f;
//    simd::float4x4 projectionMatrix  = matrix_perspective_left_hand(_fov, aspect, _nearPlane, _farPlane);
  
    
    Uniforms uniforms;
    
    simd::float4x4 modelView = viewMatrix_ljl*viewMatrix * modelMatrix;
    uniforms.modelViewMatrix = modelView;
    
    simd::float4x4 modelViewProj = projectionMatrix * modelView;
    uniforms.modelViewProjectionMatrix = modelViewProj;
    
    simd::float3x3 normalMatrix = { modelView.columns[0].xyz, modelView.columns[1].xyz, modelView.columns[2].xyz };
//    simd::float3 n1 = {1.0f,0.0f,0.0f},n2 = {0.0f,1.0f,0.0f},n3 = {0.0f,0.0f,1.0f};
//    simd::float3x3 normalMatrix = { n1,n2 , n3 };
    uniforms.normalMatrix = simd::transpose(simd::inverse(normalMatrix));
    
    _uniformBuffer = [_renderer newBufferWithBytes:(void *)&uniforms length:sizeof(Uniforms)];
    
    [self redraw];

}


- (void)drawObjectWith3:(const Mat&)R andT:(const Mat&)T {
    return ;
    float qx,qy,qz,qw;
    qw = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
    qx = (R.at<float>(2,1) - R.at<float>(1,2)) / (4*qw) ;
    qy = -(R.at<float>(0,2) - R.at<float>(2,0)) / (4*qw) ;
    qz = -(R.at<float>(1,0) - R.at<float>(0,1)) / (4*qw) ;
    
    //Mat T_p = R.t()*T;
    
    //simd::float4 r1 = { 1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, 0 };
    //simd::float4 r2 = { 2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, 0 };
    //simd::float4 r3 = { 2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0 };
    simd::float4 r1 = { 1 - 2*qy*qy - 2*qz*qz, 2*qx*qy + 2*qz*qw, 2*qx*qz - 2*qy*qw, 0 };
    simd::float4 r2 = { 2*qx*qy - 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz + 2*qx*qw, 0 };
    simd::float4 r3 = { 2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0 };
    simd::float4 r4 = { T.at<float>(0), -T.at<float>(1), -T.at<float>(2), 1 };
    
    //simd::float4 r1 = { R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), 0 };
    //simd::float4 r2 = { R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), 0 };
    //simd::float4 r3 = { R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), 0 };
    //simd::float4 r4 = { -T.at<float>(0), T.at<float>(1), T.at<float>(2), 1 };
    
    simd::float4x4 poseMatrix = {r1, r2, r3, r4};
    simd::float4x4 modelMatrix = poseMatrix*matrix_uniform_scale(0.3);
    
    simd::float4x4 viewMatrix = Identity();
    //viewMatrix.columns[3].z = -0.5; // translate camera back along Z axis
    
    const float near = 0.02;
    const float far = 100;
    const float aspect = self.metalARView.bounds.size.width / self.metalARView.bounds.size.height;
    simd::float4x4 projectionMatrix = PerspectiveProjection(aspect, DegToRad(75), near, far);
    
    Uniforms uniforms;
    
    simd::float4x4 modelView =  viewMatrix*modelMatrix;
    uniforms.modelViewMatrix = modelView;
    
    simd::float4x4 modelViewProj = projectionMatrix * modelView;
    uniforms.modelViewProjectionMatrix = modelViewProj;
    
    simd::float3x3 normalMatrix = { modelView.columns[0].xyz, modelView.columns[1].xyz, modelView.columns[2].xyz };
    uniforms.normalMatrix = simd::transpose(simd::inverse(normalMatrix));
    
    _uniformBuffer = [_renderer newBufferWithBytes:(void *)&uniforms length:sizeof(Uniforms)];
    
    [self redraw];
}


- (void)redraw
{
//    return ;
    [_renderer startFrame];
    
    [_renderer drawTrianglesWithInterleavedBuffer:_vertexBuffer indexBuffer:_indexBuffer uniformBuffer:_uniformBuffer indexCount:[_indexBuffer length]/sizeof(IndexType)];
    
    [_renderer endFrame];
}

@end

