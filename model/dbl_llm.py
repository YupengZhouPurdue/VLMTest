from langchain_openai import ChatOpenAI
from langchain.llms import OpenAI,OpenAIChat
import replicate
import google.generativeai as palm
import PIL.Image
import base64
from openai import OpenAI
# Only import OpenAI from the official SDK when you need to use the vision API:
# (do NOT import OpenAI from langchain or langchain_openai)

# from output_parsers.simple import SimpleParser


class DBL:
    def __init__(self,
                 llm_model_name: str = 'gpt-4',
                 template_name: str = 'basic_demo',
                 memory_enable: bool = False,
                 memory_path: str = 'memory'
                 ):
        self.llm_modal_name = llm_model_name
        if llm_model_name == 'gpt-4':
            self.llm = ChatOpenAI(
                model_name='gpt-4',
                temperature=0.7,
                max_tokens=512,
            )
        elif llm_model_name == 'gpt-4v':
            self.llm = ChatOpenAI(
                model_name='gpt-4o',
                temperature=0.7,
                max_tokens=512,
            )
        elif llm_model_name == 'gpt-4-turbo':
            self.llm = OpenAI(
                model_name='gpt-4-1106-preview',
                temperature=0.7,
                max_tokens=512
            )
        elif llm_model_name == 'gpt-3.5-turbo':
            self.llm = ChatOpenAI(
                model_name='gpt-3.5-turbo-1106',
                temperature=0.7,
                max_tokens=512,
            )
        elif llm_model_name == 'gpt-3':
            self.llm = ChatOpenAI(
                model_name='gpt-3',
                temperature=0.7,
                max_tokens=512,
            )
        elif llm_model_name == 'llama-2-70b':
            self.llm = 'llama-2-70b'
        elif llm_model_name == 'palm-2':
            self.llm = 'palm-2'
        elif llm_model_name == 'gemini-pro-vision':
            self.llm = 'gemini-pro-vision'

        elif llm_model_name == 'text-davinci-003':
            self.llm = OpenAI(
                model_name='text-davinci-003',
                temperature=0,
                max_tokens=256
            )
        else:
            raise ValueError(f'Unsupported LLM model name: {llm_model_name}')
        with open(f'templates/{template_name}.txt', 'r') as f:
            self.template = ''.join(f.readlines())
            if memory_enable:
                with open(memory_path, 'r') as file:
                    self.template += ''.join(file.readlines())

    def run(self, query: str):
        prompt = '\n'.join([self.template, query])
        if self.llm == 'llama-2-70b':
            output = replicate.run(
                "meta/llama-2-70b:a52e56fee2269a78c9279800ec88898cecb6c8f1df22a6483132bea266648f00",
                input={
                    "prompt": prompt,
                    "temperature": 0.75,
                    "max_new_tokens": 800,
                    "min_new_tokens": -1
                }
            )
            result = ''
            for i in output:
                print(i)
                result.join(i)
        elif self.llm == 'palm-2':
            palm.configure(api_key='YOUR_API_KEY')
            models = [m for m in palm.list_models() if 'generateText' in m.supported_generation_methods]
            model = models[0].name
            completion = palm.generate_text(
                model=model,
                prompt=prompt,
                temperature=0,
                # The maximum length of the response
                max_output_tokens=800,
            )
            result = completion.result
        elif self.llm_modal_name == 'gpt-4v':
            import os
            import base64
            image_path = "/home/yupeng/Documents/DSMD/VLM_test_platform/perception_aggregator/camera_lf_image.jpg"
            with open(image_path, "rb") as image_file:
                base64_image = base64.b64encode(image_file.read()).decode("utf-8")
            data_url = f"data:image/jpeg;base64,{base64_image}"

            client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            result = client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": data_url
                                }
                            },
                        ],
                    }
                ],
                max_tokens=300,
            )
        elif self.llm_modal_name == 'gemini-pro-vision':
            model = palm.GenerativeModel('gemini-pro-vision')
            img = PIL.Image.open('/home/yupeng/Documents/DSMD/VLM_test_platform/perception_aggregator/camera_lf_image.jpg')
            response = model.generate_content([prompt, img])
        else:
            # prompt = '\n'.join([self.template, query])
            result = self.llm(prompt) if isinstance(self.llm, OpenAI) or isinstance(self.llm, OpenAIChat) else self.llm.call_as_llm(prompt)
        return result