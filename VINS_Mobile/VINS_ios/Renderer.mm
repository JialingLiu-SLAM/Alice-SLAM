//
//  Renderer.cpp
//  VINS_ios
//
//  Created by 张剑华 on 2023/5/8.
//  Copyright © 2023 栗大人. All rights reserved.
//

#import "Renderer.h"
#include <iostream>

@interface Renderer ()

@property (nonatomic, getter=pipelineIsDirty) BOOL pipelineDirty;

// Long-lived objects
@property (nonatomic, strong) CAMetalLayer *metalLayer;
@property (nonatomic, strong) id<MTLCommandQueue> commandQueue;
@property (nonatomic, strong) id<MTLDevice> device;
@property (nonatomic, strong) id<MTLLibrary> library;

// Lazily recreated objects
@property (nonatomic, strong) id<MTLRenderPipelineState> pipeline;
@property (nonatomic, strong) id<MTLDepthStencilState> depthStencilState;

// Per-frame transient objects
@property (nonatomic, strong) id<MTLRenderCommandEncoder> commandEncoder;
@property (nonatomic, strong) id<MTLCommandBuffer> commandBuffer;
@property (nonatomic, strong) id<CAMetalDrawable> drawable;
@end

@implementation Renderer

@synthesize vertexFunctionName=_vertexFunctionName;
@synthesize fragmentFunctionName=_fragmentFunctionName;

- (instancetype)initWithLayer:(CAMetalLayer *)metalLayer
{
    if ((self = [super init]))
    {
        _metalLayer = metalLayer;
        _device = MTLCreateSystemDefaultDevice();//获取默认的device
        if (!_device)
        {
            NSLog(@"Unable to create default device!");
        }
        _metalLayer.device = _device;
        _metalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
        
        _library = [_device newDefaultLibrary];//.metal
        
        _pipelineDirty = YES;
    }
    
    return self;
}

- (NSString *)vertexFunctionName
{
    return _vertexFunctionName;
}

- (void)setVertexFunctionName:(NSString *)vertexFunctionName
{
    self.pipelineDirty = YES;
    
    _vertexFunctionName = [vertexFunctionName copy];
}

- (NSString *)fragmentFunctionName
{
    return _fragmentFunctionName;
}

- (void)setFragmentFunctionName:(NSString *)fragmentFunctionName
{
    self.pipelineDirty = YES;
    
    _fragmentFunctionName = [fragmentFunctionName copy];
}
//设置渲染管道
- (void)buildPipeline
{
    MTLVertexDescriptor *vertexDescriptor = [MTLVertexDescriptor vertexDescriptor];
    vertexDescriptor.attributes[0].format = MTLVertexFormatFloat4;
    vertexDescriptor.attributes[0].bufferIndex = 0;
    vertexDescriptor.attributes[0].offset = 0;
    
    vertexDescriptor.attributes[1].format = MTLVertexFormatFloat4;
    vertexDescriptor.attributes[1].bufferIndex = 0;
    vertexDescriptor.attributes[1].offset = sizeof(float) * 4;//4

    vertexDescriptor.layouts[0].stride = sizeof(float) * 8;//8
    vertexDescriptor.layouts[0].stepFunction = MTLVertexStepFunctionPerVertex;
    
    MTLRenderPipelineDescriptor *pipelineDescriptor = [MTLRenderPipelineDescriptor new];
    pipelineDescriptor.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
    pipelineDescriptor.vertexFunction = [self.library newFunctionWithName:self.vertexFunctionName];//顶点shader vertexShader是函数名
    pipelineDescriptor.fragmentFunction = [self.library newFunctionWithName:self.fragmentFunctionName];//片元shader samplingShader是函数名
    pipelineDescriptor.vertexDescriptor = vertexDescriptor;
    
//    std::cout<<"名字是否错误"<<self.vertexFunctionName<<" , "<< self.fragmentFunctionName<<std::endl;
    
    //创建深度缓存
    MTLDepthStencilDescriptor *depthStencilDescriptor = [MTLDepthStencilDescriptor new];
    depthStencilDescriptor.depthCompareFunction = MTLCompareFunctionLess;
    depthStencilDescriptor.depthWriteEnabled = YES;
    self.depthStencilState = [self.device newDepthStencilStateWithDescriptor:depthStencilDescriptor];

    NSError *error = nil;
    self.pipeline = [self.device newRenderPipelineStateWithDescriptor:pipelineDescriptor
                                                                error:&error];//创建图像渲染管道，耗性能操作不宜频繁调用
    
    if (!self.pipeline)
    {
        NSLog(@"Error occurred when creating render pipeline state: %@", error);
    }
    
    self.commandQueue = [self.device newCommandQueue];//渲染指令队列 保证渲染指令有序地提交到GPU
}

- (id<MTLBuffer>)newBufferWithBytes:(const void *)bytes length:(NSUInteger)length
{
    //创建顶点缓存
    return [self.device newBufferWithBytes:bytes
                                    length:length
                                   options:MTLResourceOptionCPUCacheModeDefault];
}

- (void)startFrame
{
    self.drawable = [self.metalLayer nextDrawable];
    id<MTLTexture> framebufferTexture = self.drawable.texture;
    
    if (!framebufferTexture)
    {
        NSLog(@"Unable to retrieve texture; drawable may be nil");
        return;
    }

    
//    MTLTextureDescriptor *depthTexDesc=[MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float width:self.drawable.texture.width height:self.drawable.texture.height mipmapped:NO];
//    id<MTLTexture> depthTex=[_device newTextureWithDescriptor:depthTexDesc];
    
    if (self.pipelineIsDirty)
    {
        [self buildPipeline];
        self.pipelineDirty = NO;
    }
    
    //render des
    MTLRenderPassDescriptor *renderPass = [MTLRenderPassDescriptor renderPassDescriptor];
    renderPass.colorAttachments[0].texture = framebufferTexture;
    renderPass.colorAttachments[0].clearColor = MTLClearColorMake(0.0, 0.0, 0.0, 0.0);//背景颜色区域 是整个区域5，0，0，0
    renderPass.colorAttachments[0].storeAction = MTLStoreActionStore;
    renderPass.colorAttachments[0].loadAction = MTLLoadActionClear;
//    renderPass.depthAttachment.texture=depthTex;
//    renderPass.depthAttachment.loadAction=MTLLoadActionClear;
//    renderPass.depthAttachment.storeAction=MTLStoreActionDontCare;
    
    //command encoder
    self.commandBuffer = [self.commandQueue commandBuffer];
    self.commandEncoder = [self.commandBuffer renderCommandEncoderWithDescriptor:renderPass];
    [self.commandEncoder setRenderPipelineState:self.pipeline];
    //然后设置深度测试
//    [self.commandEncoder setDepthStencilState:self.depthStencilState];
    //图元朝向做剔除
    [self.commandEncoder setFrontFacingWinding:MTLWindingCounterClockwise];
    [self.commandEncoder setCullMode:MTLCullModeBack];
}

- (void)drawTrianglesWithInterleavedBuffer:(id<MTLBuffer>)positionBuffer
                               indexBuffer:(id<MTLBuffer>)indexBuffer
                             uniformBuffer:(id<MTLBuffer>)uniformBuffer
                                indexCount:(size_t)indexCount
{
    if (!positionBuffer || !indexBuffer || !uniformBuffer)
    {
        return;
    }
//    printf("drawTrianglesWithInterleavedBuffer\n");
    [self.commandEncoder setVertexBuffer:positionBuffer offset:0 atIndex:0];
    [self.commandEncoder setVertexBuffer:uniformBuffer offset:0 atIndex:1];
    [self.commandEncoder setFragmentBuffer:uniformBuffer offset:0 atIndex:0];
    //set render vertex
    [self.commandEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                    indexCount:indexCount
                                     indexType:MTLIndexTypeUInt16
                                   indexBuffer:indexBuffer
                             indexBufferOffset:0];
}

- (void)endFrame
{
    [self.commandEncoder endEncoding];
    
    if (self.drawable)
    {
        [self.commandBuffer presentDrawable:self.drawable];
        [self.commandBuffer commit];
    }

}


@end
