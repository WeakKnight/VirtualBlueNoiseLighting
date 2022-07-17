# Graphs
from falcor import *

def render_graph_g():
    g = RenderGraph('g')
    loadRenderPassLibrary('SampleEliminatePass.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('ReSTIRPass.dll')
    loadRenderPassLibrary('VirtualLightVisPass.dll')
    loadRenderPassLibrary('PixelInspectorPass.dll')
    loadRenderPassLibrary('Antialiasing.dll')
    loadRenderPassLibrary('BlitPass.dll')
    loadRenderPassLibrary('BSDFReSTIRPass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('GBufferSlim.dll')
    loadRenderPassLibrary('PrepareLights.dll')
    loadRenderPassLibrary('SimplePathTracer.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    loadRenderPassLibrary('Utils.dll')
    loadRenderPassLibrary('VirtualLightEstimatePass.dll')
    loadRenderPassLibrary('VirtualLightGeneratePass.dll')
    GBufferSlim = createPass('GBufferSlim')
    g.addPass(GBufferSlim, 'GBufferSlim')
    ToneMapper = createPass('ToneMapper', {
        'exposureCompensation': 0.0, 
        'autoExposure': False, 
        'filmSpeed': 100.0, 
        'whiteBalance': False, 
        'whitePoint': 6500.0, 
        'operator': ToneMapOp.Aces, 
        'clamp': True, 
        'whiteMaxLuminance': 1.0, 
        'whiteScale': 11.199999809265137, 
        'fNumber': 1.0, 
        'shutter': 1.0, 
        'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')
    AccumulatePass = createPass('AccumulatePass', {
        'enableAccumulation': True, 
        'autoReset': True, 
        'precisionMode': AccumulatePrecision.Single, 
        'subFrameCount': 0})
    g.addPass(AccumulatePass, 'AccumulatePass')
    VirtualLightVisPass = createPass('VirtualLightVisPass', {
        'radius': 0.002000000189989805, 
        'visMode': 0, 
        'visType': 0})
    g.addPass(VirtualLightVisPass, 'VirtualLightVisPass')
    VirtualLightGeneratePass_ = createPass('VirtualLightGeneratePass', {
        'TileSize': 4, 
        'TileSampleNum': 2, 
        'boundBoxRadius': 0.006000000052154064})
    g.addPass(VirtualLightGeneratePass_, 'VirtualLightGeneratePass_')
    VirtualLightEstimatePass = createPass('VirtualLightEstimatePass', {
        'Photon Path Count': 300000, 
        'Per Frame Path Count': 20000, 
        'Texture Item Size': 16,
        'BounceNum':256, 
        'MegaTexture Capacity': 200000, 
        'MegaTexture Capacity LQ':300000, 
        'Convert Incoming To Outgoing': True})
    g.addPass(VirtualLightEstimatePass, 'VirtualLightEstimatePass')
    SampleEliminatePass = createPass('SampleEliminatePass', {
        'ratio': 0.13300000149011612, 
        'radiusSearchRange': 0.10000000298023224, 
        'radiusSearchCount': 100, 
        'radius': 0.025000001192092896, 
        'uniformSE': False, 
        'radiusScalerForASBuilding': 1.100000023841858, 
        'useDMaxForASBuilding': True, 
        'useParallelSE': True})
    g.addPass(SampleEliminatePass, 'SampleEliminatePass')
    BSDFReSTIRPass = createPass('BSDFReSTIRPass', {
        'MaxPathIntensity':0.0,
        'BounceNum':256, 
        'enableDirectLighting': True, 
        'enableSpatialResampling': False, 
        'enableTemporalResampling': False, 
        'improveCorner': True, 'shadingMode': 0, 
        'shortDistance': 0.3, 
        'shortDistanceRange':0.15, 
        'MISWithPowerSampling': True})
    g.addPass(BSDFReSTIRPass, 'BSDFReSTIRPass')
    PrepareLights = createPass('PrepareLights')
    g.addPass(PrepareLights, 'PrepareLights')
    g.addEdge('PrepareLights.output', 'BSDFReSTIRPass.input')
    g.addEdge('SampleEliminatePass.output', 'VirtualLightEstimatePass.input')
    g.addEdge('AccumulatePass.output', 'ToneMapper.src')
    g.addEdge('VirtualLightGeneratePass_.output', 'SampleEliminatePass.input')
    g.addEdge('GBufferSlim.output', 'VirtualLightGeneratePass_.input')
    g.addEdge('BSDFReSTIRPass.output', 'AccumulatePass.input')
    g.addEdge('VirtualLightEstimatePass.output', 'VirtualLightVisPass.dummy')
    g.addEdge('VirtualLightEstimatePass.output', 'PrepareLights.input')
    g.markOutput('ToneMapper.dst')
    g.markOutput('AccumulatePass.output')
    g.markOutput('VirtualLightVisPass.output')
    return g
m.addGraph(render_graph_g())

m.loadScene('EmbeddedMedia/CornellBudda/CornellBudda2.pyscene')
m.scene.renderSettings = SceneRenderSettings(useEnvLight=False, useAnalyticLights=True, useEmissiveLights=True, useVolumes=False)
m.scene.camera.position = float3(0.408790,0.966490,2.346600)
m.scene.camera.target = float3(0.345850,0.881480,1.352185)
m.scene.camera.up = float3(0.000065,0.999900,0.001105)
m.scene.cameraSpeed = 1.0

# Window Configuration
m.resizeSwapChain(1600, 900)

m.ui = True

# Clock Settings
m.clock.time = 0
m.clock.framerate = 0

# Frame Capture
m.frameCapture.outputDir = '.'
m.frameCapture.baseFilename = 'Mogwai'

