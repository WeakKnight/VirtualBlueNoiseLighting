# Virtual Blue Noise Lighting (VBNL)

![](Teaser.png)

## Introduction
- This repo includes the source code for the following HPG 2022 paper

> **Virtual Blue Noise Lighting**<br>
> Tianyu Li*, Wenyou Wang*, Daqi Lin, Cem Yuksel<br>
> (*Joint first authors) <br>
> https://graphics.cs.utah.edu/research/projects/virtual-blue-noise-lighting/

Virtual Blue Noise Lighting is a rendering pipeline for estimating indirect illumination with a blue noise distribution of virtual lights. Our pipeline is designed for virtual lights with non-uniform emission profiles that are more expensive to store, but required for properly and efficiently handling specular transport.

Unlike the typical virtual light placement approaches that traverse light paths from the original light sources, we generate them starting from the camera. This avoids two important problems: wasted memory and computation with fully-occluded virtual lights, and excessive virtual light density around high-probability light paths. In addition, we introduce a parallel and adaptive sample elimination strategy to achieve a blue noise distribution of virtual lights with varying density. This addresses the third problem of virtual light placement by ensuring that they are not placed too close to each other, providing better coverage of the (indirectly) visible surfaces and further improving the quality of the final lighting estimation.

For computing the virtual light emission profiles, we present a photon splitting technique that allows efficiently using a large number of photons, as it does not require storing them. During lighting estimation, our method allows using both global power-based and local BSDF important sampling techniques, combined via multiple importance sampling. In addition, we present an adaptive path extension method that avoids sampling nearby virtual lights for reducing the estimation error.

We show that our method significantly outperforms path tracing and prior work in virtual lights in terms of both performance and image quality, producing a fast but biased estimate of global illumination.

## Prerequisites
- Windows 10 version 20H2 or newer
- Visual Studio 2019
- Windows 10 SDK version 10.0.19041.1 Or Newer
- NVIDIA driver 466.11 or newer
- RTX 2060 or Higher (NVIDIA graphics card with raytracing support)
- Get NVAPI, head over to https://developer.nvidia.com/nvapi and download the latest version of NVAPI. Create a folder called `.packman` under `Source/Externals`, Extract the content of the zip file into `Source/Externals/.packman/` and rename `Rxxx-developer` to `nvapi`.

## How to compile
- Make sure you have NVAPI in `Source/Externals/.packman/` 
- Open Falcor.sln and Build Solution in configuration ReleaseD3D12

## Run the demo
- execute `RunTeaserImageOurs.bat`
