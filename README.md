# PEACE :v:: Prompt Engineering Automation for CLIPSeg Enhancement for Safe-Landing Zone Segmentation

This work is an extenion of DOVESEI (https://arxiv.org/abs/2308.11471), where we improved on the prompt generation and engineering inside DOVESEI, and improved on merging segmentation by stacking target and non-target segmemnntations. The objective was to generate prompts that are dynamic, such that prompts are adaptive to observed images instead of a static prompt.

Details about DOVESEI: https://github.com/MISTLab/DOVESEI/blob/main/README.md  <br />
PEACE enables the possibility of dynamically generating prompts that are specifically optimized for an input image. We believe that this is an important step towards developing more robust autonomous UAV systems. In summary, our main contributions are:
1) Dynamic aerial prompt engineering per image frame that can adapt to changing environments during safe-landing zone segmentation.
<img src = "assets/PEACE_DP.svg" alt = "Figure 1: PEACE Data Pipeline" width = "100%">
2) Introduce the method of combining positive and negative term segmentations to improve safe-landing zone segmentation accuracy: Generate segmentations for all safe-landing zones (positive terms: all target classes that are considered safe to land such as grass, park, water, etc.) and unsafe-landing zones (negative terms: all classes that are unsafe to land such as concrete, street, building, etc.). Segmentations of positive terms are stacked to merge safe-landing zone segmentations and the segmentations of negative terms are also stacked and merged to segment all unsafe-landing zones. Finally, the merged segmentations of negative terms are used to eliminate unsafe landing zones from the merged segmentations of positive terms.
<img src = "assets/PEACE_Seg.svg" alt = "Figure 2: PEACE Data Pipeline" width = "100%">

## Comparison
<img src = "assets/comparison.png" alt = "Figure 3: Segmetation Difference" width = "100%">

Comparison of CLIP and CLIPSeg’s original prompt engineering and PEACE using images from [CARLA](https://carla.org/). <br />

a) A photo of grass <br />
b) A blurry photo of grass in autumn <br />
c) A photo of grass <br />
d) A 3D photo of grass in morning <br />

### Testing using DOVESEI docker run --runtime nvidia -it --rm --network=host --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -e DISPLAY=$DISPLAY -v $(pwd):/home haechanmarkbong/blabberseg </br>
git clone --recurse-submodules https://github.com/MISTLab/PEACE.git </br>
sudo apt-get update  </br>
sudo apt-get upgrade </br>
colcon build --symlink-install --packages-select ros2_satellite_aerial_view_simulator ros2_open_voc_landing_heatmap ros2_open_voc_landing_heatmap_srv </br>
source install/setup.bash </br>
./experiements.bash </br>

## Publication
For more information about PEACE, refer to our paper: https://arxiv.org/abs/2310.00085
