# Graphs
from falcor import *

def render_graph_UnifiedRichVXL():
    g = RenderGraph('UnifiedRichVXL')
    loadRenderPassLibrary('SampleEliminatePass.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('VirtualLightVisPass.dll')
    loadRenderPassLibrary('PixelInspectorPass.dll')
    loadRenderPassLibrary('Antialiasing.dll')
    loadRenderPassLibrary('BlitPass.dll')
    loadRenderPassLibrary('BSDFReSTIRPass.dll')
    loadRenderPassLibrary('VirtualLightEstimatePass.dll')
    loadRenderPassLibrary('PrepareLights.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('GBufferSlim.dll')
    loadRenderPassLibrary('VPLReSTIRPass.dll')
    loadRenderPassLibrary('RelaxationPass.dll')
    loadRenderPassLibrary('Utils.dll')
    loadRenderPassLibrary('PassLibraryTemplate.dll')
    loadRenderPassLibrary('PhotonGenerationPass.dll')
    loadRenderPassLibrary('PhotonMapping.dll')
    loadRenderPassLibrary('ReSTIRPass.dll')
    loadRenderPassLibrary('SimplePathTracer.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    loadRenderPassLibrary('VirtualLightGeneratePass.dll')
    loadRenderPassLibrary('WhittedRayTracer.dll')
    PrepareLights = createPass('PrepareLights')
    g.addPass(PrepareLights, 'PrepareLights')
    GBufferSlim = createPass('GBufferSlim')
    g.addPass(GBufferSlim, 'GBufferSlim')
    PhotonGenerationPass = createPass('PhotonGenerationPass', {'PhotonPathCount': 300000, 'state': 1, 'stateStep': 1, 'BounceNum': 256, 'ComputeVSLRadius': False, 'Radius': 0.014999999664723873, 'KNNRadiusNeighbour': 5})
    g.addPass(PhotonGenerationPass, 'PhotonGenerationPass')
    RelaxationPass = createPass('RelaxationPass', {'MinPrs':0.01,'targetCount': 2000, 'radius': 0.029999999329447746, 'radiusScalerForASBuilding': 0.5, 'useDMaxForASBuilding': True, 'Texture Item Size': 16, 'MegaTexture Capacity': 200000, 'Texture Item Size LQ': 8, 'MegaTexture Capacity LQ': 100000, 'Relaxation Factor': 1.5, 'standardEnrich': True, 'KNNRadiusScaler': 10})
    g.addPass(RelaxationPass, 'RelaxationPass')
    BSDFReSTIRPass = createPass('BSDFReSTIRPass', {'shortDistance': 0.0, 'shortDistanceRange': 0.0, 'MaxPathIntensity': 0.0, 'BounceNum': 256, 'VirtualLightUseNEE': True, 'EnablePreciseVirtualLightNEE': False, 'VirtualLightUseMIS': False, 'RoughnessThreshold': 0.11999999731779099, 'UseRIS': False, 'NEESamples': 1, 'enableSpatialResampling': False, 'enableTemporalResampling': False, 'shadingMode': 2, 'MISWithPowerSampling': False, 'FinalMISWithPowerSampling': False, 'ShadingNormalCheck': True, 'ShadingMaterialCheck': False, 'UseTraditionalVXL': True, 'UseRichVXL': True, 'UseVSL': True})
    g.addPass(BSDFReSTIRPass, 'BSDFReSTIRPass')
    ToneMapper = createPass('ToneMapper', {'exposureCompensation': 0.0, 'autoExposure': False, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Aces, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')
    AccumulatePass = createPass('AccumulatePass', {'enableAccumulation': True, 'autoReset': True, 'precisionMode': AccumulatePrecision.Single, 'subFrameCount': 0, 'enableSubFrameCount': False})
    g.addPass(AccumulatePass, 'AccumulatePass')
    VirtualLightGeneratePass = createPass('VirtualLightGeneratePass', {'TileSize': 1, 'TileSampleNum': 16, 'boundBoxRadius': 0.006000000052154064, 'StartFromLights': False, 'PhotonPathCount': 20000, 'DirectLightingOnly': True, 'StartFromView': True, 'NeedRelaxation': False})
    g.addPass(VirtualLightGeneratePass, 'VirtualLightGeneratePass')
    VirtualLightVisPass = createPass('VirtualLightVisPass', {'radius': 0.004000000189989805, 'visMode': 0, 'visType': 0, 'enable': False})
    g.addPass(VirtualLightVisPass, 'VirtualLightVisPass')
    g.addEdge('PrepareLights.output', 'VirtualLightGeneratePass.input')
    g.addEdge('GBufferSlim.output', 'PrepareLights.input')
    g.addEdge('PhotonGenerationPass.output', 'RelaxationPass.input')
    g.addEdge('RelaxationPass.output', 'BSDFReSTIRPass.input')
    g.addEdge('BSDFReSTIRPass.output', 'AccumulatePass.input')
    g.addEdge('AccumulatePass.output', 'ToneMapper.src')
    g.addEdge('VirtualLightGeneratePass.output', 'PhotonGenerationPass.input')
    g.addEdge('AccumulatePass.output', 'VirtualLightVisPass.dummy')
    g.markOutput('ToneMapper.dst')
    g.markOutput('AccumulatePass.output')
    g.markOutput('VirtualLightVisPass.output')
    return g
m.addGraph(render_graph_UnifiedRichVXL())

# Scene
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
# If framerate is not zero, you can use the frame property to set the start frame
# m.clock.frame = 0

# Frame Capture
m.frameCapture.outputDir = '.'
m.frameCapture.baseFilename = 'Mogwai'

