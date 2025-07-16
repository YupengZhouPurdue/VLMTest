# A Hierarchical Test Platform for Vision Language Model (VLM)-Integrated Real-World Autonomous Driving
This repo is the case study implementation of the porposed framework in [A Hierarchical Test Platform for Vision Language Model (VLM)-Integrated Real-World Autonomous Driving](https://arxiv.org/abs/2506.14100). 

<!-- PROJECT ILLUSTRATIONS -->

<br />
<div align="center">
    <p align="center">
        <img src="VLMTest/structure.png", alt="intro", width="600"/>
    </p>
</div>

## About
Vision-Language Models (VLMs) have demonstrated significant promise for autonomous driving due to their powerful multimodal reasoning capabilities. However, adapting VLMs from generic data to safety-critical driving contexts introduces a notable challenge known as domain shift. Existing simulation-based and dataset-driven evaluation approaches struggle to accurately replicate real-world complexities, lacking repeatable closed-loop evaluation and flexible scenario manipulation. Furthermore,current real-world testing platforms typically focus on isolated modules and do not support comprehensive interaction with VLM-based systems. Consequently, there is a critical need for a holistic testing architecture capable of integrating perception, planning, and control modules, accommodating VLM-based systems and supporting configurable real-world testing scenarios.

In this paper, we address this critical gap by proposing a hierarchical real-world test platform specialized in the rigorous evaluation of VLM-integrated autonomous driving systems. Specifically, our platform features have: a lightweight, structured, and low-latency middleware pipeline specialized for seamless VLM integration; a hierarchical modular architecture enabling flexible substitution between conventional and VLM-based autonomy components, providing exceptional deployment flexibility for rapid experimentation; and sophisticated closed-loop scenario-based testing capabilities on a controlled test track, facilitating comprehensive evaluation of the entire VLM-enabled decision-making pipeline—from multimodal perception and reasoning to final vehicle maneuvers. Through an extensive real-world case study, we demonstrate the effectiveness of our platform in evaluating the performance and robustness of VLM-integrated autonomous driving under diverse realistic conditions. 

## Installation

A Python version of 3.8+ is recommended.

Install dependencies
```
pip install -r requirements.txt
```

You will also need to install PyAudio. For instructions, see https://pypi.org/project/PyAudio/.

[Openai API](https://openai.com/index/openai-api/) key is required. Either put it in `main.py` or set `OPENAI_API_KEY` in your environment variable.

To enable driving context info, [OpenWeather API](https://openweathermap.org/api) and [TomTom API](https://developer.tomtom.com/knowledgebase/platform/articles/how-to-get-an-tomtom-api-key/) keys are required. The API keys should be put into `utils/get_driving_context.py`.

(Optional) The hotword detection feature is implemented using the [EfficientWord-Net](https://github.com/Ant-Brain/EfficientWord-Net) library. You can install it using:
```
pip install EfficientWord-Net
```
A `--no-deps` option is recommended if you encounter errors during the installation. Please refer to the [EfficientWord-Net](https://github.com/Ant-Brain/EfficientWord-Net) repo for installation and few-show hotword training. 

## Usage

To execute specific actions, you need to edit the files in the `scripts/` folder. Sample bash scripts are provided for reference.

For quick start, simply run `python main.py`. For all available options, run  `python main.py -h`.

### Templates

Templates consists of pre-constructed instructions for LLMs and historical interactions between human and LLMs. Some sample templates used in the experiment can be found at the `templates/` folder. You can create your own templates and use them with the `--template_name` and `--memory_path` arguments.

## Citatation

If you find Talk2Drive useful, please consider citing our paper:

```
@misc{zhou2025hierarchicaltestplatformvision,
      title={A Hierarchical Test Platform for Vision Language Model (VLM)-Integrated Real-World Autonomous Driving}, 
      author={Yupeng Zhou and Can Cui and Juntong Peng and Zichong Yang and Juanwu Lu and Jitesh H Panchal and Bin Yao and Ziran Wang},
      year={2025},
      eprint={2506.14100},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2506.14100}, 
}
```

## License

This project is distributed under the MIT License. See [`LICENSE`](LICENSE) for more information.

## Acknowledgement

This project draws inspiration from the following open-source projects and resources:

- [Autoware-AI](https://github.com/autowarefoundation/autoware/tree/autoware-ai)
- [EfficientWord-Net](https://github.com/Ant-Brain/EfficientWord-Net)
