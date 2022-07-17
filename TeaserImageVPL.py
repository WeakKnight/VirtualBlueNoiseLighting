from falcor import *

def render_graph_DefaultRenderGraph():
    g = RenderGraph('DefaultRenderGraph')
    loadRenderPassLibrary('SceneDebugger.dll')
    loadRenderPassLibrary('BSDFViewer.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('ModulateIllumination.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('VirtualLightVisPass.dll')
    loadRenderPassLibrary('Antialiasing.dll')
    loadRenderPassLibrary('GBufferSlim.dll')
    loadRenderPassLibrary('BlitPass.dll')
    loadRenderPassLibrary('DebugPasses.dll')
    loadRenderPassLibrary('BSDFReSTIRPass.dll')
    loadRenderPassLibrary('CSM.dll')
    loadRenderPassLibrary('DirectLighting.dll')
    loadRenderPassLibrary('VirtualLightGeneratePass.dll')
    loadRenderPassLibrary('ErrorMeasurePass.dll')
    loadRenderPassLibrary('PixelInspectorPass.dll')
    loadRenderPassLibrary('ForwardLightingPass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('VirtualLightEstimatePass.dll')
    loadRenderPassLibrary('ResultAccumulatePass.dll')
    loadRenderPassLibrary('ImageLoader.dll')
    loadRenderPassLibrary('MegakernelPathTracer.dll')
    loadRenderPassLibrary('MinimalPathTracer.dll')
    loadRenderPassLibrary('PassLibraryTemplate.dll')
    loadRenderPassLibrary('PhotonGenerationPass.dll')
    loadRenderPassLibrary('PhotonMapping.dll')
    loadRenderPassLibrary('RelaxationPass.dll')
    loadRenderPassLibrary('PrepareLights.dll')
    loadRenderPassLibrary('ReSTIRPass.dll')
    loadRenderPassLibrary('RISPass.dll')
    loadRenderPassLibrary('SampleEliminatePass.dll')
    loadRenderPassLibrary('SimplePathTracer.dll')
    loadRenderPassLibrary('SkyBox.dll')
    loadRenderPassLibrary('SSAO.dll')
    loadRenderPassLibrary('SVGFPass.dll')
    loadRenderPassLibrary('TemporalDelayPass.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    loadRenderPassLibrary('Utils.dll')
    loadRenderPassLibrary('VPLReSTIRPass.dll')
    loadRenderPassLibrary('WhittedRayTracer.dll')
    GBufferSlim = createPass('GBufferSlim')
    g.addPass(GBufferSlim, 'GBufferSlim')
    BSDFReSTIRPass = createPass('BSDFReSTIRPass', {'shortDistance': 0.05000000074505806, 'shortDistanceRange': 0.009999999776482582, 'MaxPathIntensity': 0.0, 'BounceNum': 256, 'VirtualLightUseNEE': True, 'EnablePreciseVirtualLightNEE': False, 'VirtualLightUseMIS': True, 'RoughnessThreshold': 0.11999999731779099, 'UseRIS': False, 'NEESamples': 1, 'enableSpatialResampling': False, 'enableTemporalResampling': False, 'shadingMode': 2, 'MISWithPowerSampling': False, 'FinalMISWithPowerSampling': False, 'ShadingNormalCheck': True, 'ShadingMaterialCheck': False, 'UseTraditionalVXL': True, 'UseRichVXL': False, 'UseVSL': False})
    g.addPass(BSDFReSTIRPass, 'BSDFReSTIRPass')
    ToneMapper = createPass('ToneMapper', {'exposureCompensation': 0.0, 'autoExposure': False, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Aces, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')
    PhotonGenerationPass = createPass('PhotonGenerationPass', {'PhotonPathCount': 3000, 'state': -1, 'stateStep': 4, 'BounceNum': 256, 'ComputeVSLRadius': True, 'Radius': 0.014999999664723873, 'KNNRadiusNeighbour': 5})
    g.addPass(PhotonGenerationPass, 'PhotonGenerationPass')
    AccumulatePass = createPass('AccumulatePass', {'enableAccumulation': True, 'autoReset': True, 'precisionMode': AccumulatePrecision.Single, 'subFrameCount': 0, 'enableSubFrameCount': False})
    g.addPass(AccumulatePass, 'AccumulatePass')
    PrepareLights = createPass('PrepareLights')
    g.addPass(PrepareLights, 'PrepareLights')
    g.addEdge('PhotonGenerationPass.output', 'BSDFReSTIRPass.input')
    g.addEdge('BSDFReSTIRPass.output', 'AccumulatePass.input')
    g.addEdge('AccumulatePass.output', 'ToneMapper.src')
    g.addEdge('GBufferSlim.output', 'PrepareLights.input')
    g.addEdge('PrepareLights.output', 'PhotonGenerationPass.input')
    g.markOutput('ToneMapper.dst')
    return g

DefaultRenderGraph = render_graph_DefaultRenderGraph()
try: m.addGraph(DefaultRenderGraph)
except NameError: None

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

