---
title: 如何使用ChatGPT的API(五)链式提示
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: b97c365d
date: 2023-09-30 20:08:16
---

本篇文章介绍如何通过将多个提示词结合起来，将复杂的任务分割成一系列更简单的子任务。



对于一个复杂的任务，我们可以通过一个复杂的提示词，让模型进行思维链推理，从而一步一步处理复杂任务。另一方面，我们也可以将一个复杂任务分成多个子任务，每个子任务都由一个单独的prompt去驱动模型处理。这样组织prompt的形式成为链式提示。



那么在什么情况下，我们需要将一个复杂任务拆分成多个子任务来处理呢？

<!--more-->

我们用做一道复杂的菜肴来类比。一个长而复杂的prompt就像试图一次性做完一道复杂的菜肴。你必须同时管理多种原料、烹饪和拿捏时间。要跟踪所有的东西并确保每个部分都被完美地烹制，这可能是一个比较大的挑战。另一方面，**链式提示**就像给烹饪菜肴分阶段，你一次专注于一个组成部分，确保每个部分都被正确切割，然后再进入下一个阶段。这种方法打破了任务的复杂性，使其更容易管理，减少了出错的可能性。然而，对于一个非常简单的菜肴来说，这种方法是不必要的，而且过于复杂。



一个更好一点的类比是所有东西都在一个长文件里的意大利面条式代码和一个模块化代码之间的对比。使意大利面条式代码变得糟糕并难以调试的是逻辑的不同部分之间的模糊性和复杂的依赖性。而对于提交给语言模型的复杂单次任务，也会出现这种情况。



对于一个用户的提问，第一步我们可以对用户的提问进行分类并获取任务的上下文信息，然后每个类别都对应不同的子任务。第二步，基于第一步中获取到的上下文信息结合prompt处理对应子任务。这种方式确保模型拥有执行任务所需的必要信息而没有过多的不相关信息，并减少错误的可能性。



另一方面，这种方法可以降低成本。因为更长的提示，意味着更多的Token，系统运行成本更高。



这种方法的另一个好处是，更容易测试排查问题。同时也更方便将人的反馈或操作放到相应的任务流程中。



如果有必要，这种方式还允许模型在工作流程的某些点上使用外部工具。例如，它可能决定在产品目录中查找一些信息，或调用API，或搜索知识库，这些都是单一提示所不能实现的。



所以，与其在一个prompt中用几十个要点或几段话来描述整个复杂的工作流程，不如将其划分成多个子任务，从外部跟踪每个子任务的状态，然后根据需要注入相关指令和上下文信息。



下 面我们通过一个用户询问产品信息的例子更好地理解这个方法。

### 提取相关的产品和类别名称

```python
delimiter = "####"
system_message = f"""
You will be provided with customer service queries. \
The customer service query will be delimited with \
{delimiter} characters.
Output a python list of objects, where each object has \
the following format:
    'category': <one of Computers and Laptops, \
    Smartphones and Accessories, \
    Televisions and Home Theater Systems, \
    Gaming Consoles and Accessories, 
    Audio Equipment, Cameras and Camcorders>,
OR
    'products': <a list of products that must \
    be found in the allowed products below>

Where the categories and products must be found in \
the customer service query.
If a product is mentioned, it must be associated with \
the correct category in the allowed products list below.
If no products or categories are found, output an \
empty list.

Allowed products: 

Computers and Laptops category:
TechPro Ultrabook
BlueWave Gaming Laptop
PowerLite Convertible
TechPro Desktop
BlueWave Chromebook

Smartphones and Accessories category:
SmartX ProPhone
MobiTech PowerCase
SmartX MiniPhone
MobiTech Wireless Charger
SmartX EarBuds

Televisions and Home Theater Systems category:
CineView 4K TV
SoundMax Home Theater
CineView 8K TV
SoundMax Soundbar
CineView OLED TV

Gaming Consoles and Accessories category:
GameSphere X
ProGamer Controller
GameSphere Y
ProGamer Racing Wheel
GameSphere VR Headset

Audio Equipment category:
AudioPhonic Noise-Canceling Headphones
WaveSound Bluetooth Speaker
AudioPhonic True Wireless Earbuds
WaveSound Soundbar
AudioPhonic Turntable

Cameras and Camcorders category:
FotoSnap DSLR Camera
ActionCam 4K
FotoSnap Mirrorless Camera
ZoomMaster Camcorder
FotoSnap Instant Camera

Only output the list of objects, with nothing else.
"""
user_message_1 = f"""
 tell me about the smartx pro phone and \
 the fotosnap camera, the dslr one. \
 Also tell me about your tvs """
messages =  [  
{'role':'system', 
 'content': system_message},    
{'role':'user', 
 'content': f"{delimiter}{user_message_1}{delimiter}"},  
] 
category_and_product_response_1 = get_completion_from_messages(messages)
print(category_and_product_response_1)
```

回答：

```python
[{'category': 'Smartphones and Accessories', 'products': ['SmartX ProPhone']}, {'category': 'Cameras and Camcorders', 'products': ['FotoSnap DSLR Camera']}, {'category': 'Televisions and Home Theater Systems'}]
```



上面查询的产品都在产品列表上，所以模型正确地输出了产品对应的类别。由于产品列表里没有路由器，所以下面例子中，用户询问路由器的问题只会返回一个空的python list。



```python
user_message_2 = f"""
my router isn't working"""
messages =  [  
{'role':'system',
 'content': system_message},    
{'role':'user',
 'content': f"{delimiter}{user_message_2}{delimiter}"},  
] 
response = get_completion_from_messages(messages)
print(response)
```

回答：

```python
[]
```



### 检索所提取产品和类别的详细产品信息

```python
# product information
products = {
    "TechPro Ultrabook": {
        "name": "TechPro Ultrabook",
        "category": "Computers and Laptops",
        "brand": "TechPro",
        "model_number": "TP-UB100",
        "warranty": "1 year",
        "rating": 4.5,
        "features": ["13.3-inch display", "8GB RAM", "256GB SSD", "Intel Core i5 processor"],
        "description": "A sleek and lightweight ultrabook for everyday use.",
        "price": 799.99
    },
    "BlueWave Gaming Laptop": {
        "name": "BlueWave Gaming Laptop",
        "category": "Computers and Laptops",
        "brand": "BlueWave",
        "model_number": "BW-GL200",
        "warranty": "2 years",
        "rating": 4.7,
        "features": ["15.6-inch display", "16GB RAM", "512GB SSD", "NVIDIA GeForce RTX 3060"],
        "description": "A high-performance gaming laptop for an immersive experience.",
        "price": 1199.99
    },
    "PowerLite Convertible": {
        "name": "PowerLite Convertible",
        "category": "Computers and Laptops",
        "brand": "PowerLite",
        "model_number": "PL-CV300",
        "warranty": "1 year",
        "rating": 4.3,
        "features": ["14-inch touchscreen", "8GB RAM", "256GB SSD", "360-degree hinge"],
        "description": "A versatile convertible laptop with a responsive touchscreen.",
        "price": 699.99
    },
    "TechPro Desktop": {
        "name": "TechPro Desktop",
        "category": "Computers and Laptops",
        "brand": "TechPro",
        "model_number": "TP-DT500",
        "warranty": "1 year",
        "rating": 4.4,
        "features": ["Intel Core i7 processor", "16GB RAM", "1TB HDD", "NVIDIA GeForce GTX 1660"],
        "description": "A powerful desktop computer for work and play.",
        "price": 999.99
    },
    "BlueWave Chromebook": {
        "name": "BlueWave Chromebook",
        "category": "Computers and Laptops",
        "brand": "BlueWave",
        "model_number": "BW-CB100",
        "warranty": "1 year",
        "rating": 4.1,
        "features": ["11.6-inch display", "4GB RAM", "32GB eMMC", "Chrome OS"],
        "description": "A compact and affordable Chromebook for everyday tasks.",
        "price": 249.99
    },
    "SmartX ProPhone": {
        "name": "SmartX ProPhone",
        "category": "Smartphones and Accessories",
        "brand": "SmartX",
        "model_number": "SX-PP10",
        "warranty": "1 year",
        "rating": 4.6,
        "features": ["6.1-inch display", "128GB storage", "12MP dual camera", "5G"],
        "description": "A powerful smartphone with advanced camera features.",
        "price": 899.99
    },
    "MobiTech PowerCase": {
        "name": "MobiTech PowerCase",
        "category": "Smartphones and Accessories",
        "brand": "MobiTech",
        "model_number": "MT-PC20",
        "warranty": "1 year",
        "rating": 4.3,
        "features": ["5000mAh battery", "Wireless charging", "Compatible with SmartX ProPhone"],
        "description": "A protective case with built-in battery for extended usage.",
        "price": 59.99
    },
    "SmartX MiniPhone": {
        "name": "SmartX MiniPhone",
        "category": "Smartphones and Accessories",
        "brand": "SmartX",
        "model_number": "SX-MP5",
        "warranty": "1 year",
        "rating": 4.2,
        "features": ["4.7-inch display", "64GB storage", "8MP camera", "4G"],
        "description": "A compact and affordable smartphone for basic tasks.",
        "price": 399.99
    },
    "MobiTech Wireless Charger": {
        "name": "MobiTech Wireless Charger",
        "category": "Smartphones and Accessories",
        "brand": "MobiTech",
        "model_number": "MT-WC10",
        "warranty": "1 year",
        "rating": 4.5,
        "features": ["10W fast charging", "Qi-compatible", "LED indicator", "Compact design"],
        "description": "A convenient wireless charger for a clutter-free workspace.",
        "price": 29.99
    },
    "SmartX EarBuds": {
        "name": "SmartX EarBuds",
        "category": "Smartphones and Accessories",
        "brand": "SmartX",
        "model_number": "SX-EB20",
        "warranty": "1 year",
        "rating": 4.4,
        "features": ["True wireless", "Bluetooth 5.0", "Touch controls", "24-hour battery life"],
        "description": "Experience true wireless freedom with these comfortable earbuds.",
        "price": 99.99
    },

    "CineView 4K TV": {
        "name": "CineView 4K TV",
        "category": "Televisions and Home Theater Systems",
        "brand": "CineView",
        "model_number": "CV-4K55",
        "warranty": "2 years",
        "rating": 4.8,
        "features": ["55-inch display", "4K resolution", "HDR", "Smart TV"],
        "description": "A stunning 4K TV with vibrant colors and smart features.",
        "price": 599.99
    },
    "SoundMax Home Theater": {
        "name": "SoundMax Home Theater",
        "category": "Televisions and Home Theater Systems",
        "brand": "SoundMax",
        "model_number": "SM-HT100",
        "warranty": "1 year",
        "rating": 4.4,
        "features": ["5.1 channel", "1000W output", "Wireless subwoofer", "Bluetooth"],
        "description": "A powerful home theater system for an immersive audio experience.",
        "price": 399.99
    },
    "CineView 8K TV": {
        "name": "CineView 8K TV",
        "category": "Televisions and Home Theater Systems",
        "brand": "CineView",
        "model_number": "CV-8K65",
        "warranty": "2 years",
        "rating": 4.9,
        "features": ["65-inch display", "8K resolution", "HDR", "Smart TV"],
        "description": "Experience the future of television with this stunning 8K TV.",
        "price": 2999.99
    },
    "SoundMax Soundbar": {
        "name": "SoundMax Soundbar",
        "category": "Televisions and Home Theater Systems",
        "brand": "SoundMax",
        "model_number": "SM-SB50",
        "warranty": "1 year",
        "rating": 4.3,
        "features": ["2.1 channel", "300W output", "Wireless subwoofer", "Bluetooth"],
        "description": "Upgrade your TV's audio with this sleek and powerful soundbar.",
        "price": 199.99
    },
    "CineView OLED TV": {
        "name": "CineView OLED TV",
        "category": "Televisions and Home Theater Systems",
        "brand": "CineView",
        "model_number": "CV-OLED55",
        "warranty": "2 years",
        "rating": 4.7,
        "features": ["55-inch display", "4K resolution", "HDR", "Smart TV"],
        "description": "Experience true blacks and vibrant colors with this OLED TV.",
        "price": 1499.99
    },

    "GameSphere X": {
        "name": "GameSphere X",
        "category": "Gaming Consoles and Accessories",
        "brand": "GameSphere",
        "model_number": "GS-X",
        "warranty": "1 year",
        "rating": 4.9,
        "features": ["4K gaming", "1TB storage", "Backward compatibility", "Online multiplayer"],
        "description": "A next-generation gaming console for the ultimate gaming experience.",
        "price": 499.99
    },
    "ProGamer Controller": {
        "name": "ProGamer Controller",
        "category": "Gaming Consoles and Accessories",
        "brand": "ProGamer",
        "model_number": "PG-C100",
        "warranty": "1 year",
        "rating": 4.2,
        "features": ["Ergonomic design", "Customizable buttons", "Wireless", "Rechargeable battery"],
        "description": "A high-quality gaming controller for precision and comfort.",
        "price": 59.99
    },
    "GameSphere Y": {
        "name": "GameSphere Y",
        "category": "Gaming Consoles and Accessories",
        "brand": "GameSphere",
        "model_number": "GS-Y",
        "warranty": "1 year",
        "rating": 4.8,
        "features": ["4K gaming", "500GB storage", "Backward compatibility", "Online multiplayer"],
        "description": "A compact gaming console with powerful performance.",
        "price": 399.99
    },
    "ProGamer Racing Wheel": {
        "name": "ProGamer Racing Wheel",
        "category": "Gaming Consoles and Accessories",
        "brand": "ProGamer",
        "model_number": "PG-RW200",
        "warranty": "1 year",
        "rating": 4.5,
        "features": ["Force feedback", "Adjustable pedals", "Paddle shifters", "Compatible with GameSphere X"],
        "description": "Enhance your racing games with this realistic racing wheel.",
        "price": 249.99
    },
    "GameSphere VR Headset": {
        "name": "GameSphere VR Headset",
        "category": "Gaming Consoles and Accessories",
        "brand": "GameSphere",
        "model_number": "GS-VR",
        "warranty": "1 year",
        "rating": 4.6,
        "features": ["Immersive VR experience", "Built-in headphones", "Adjustable headband", "Compatible with GameSphere X"],
        "description": "Step into the world of virtual reality with this comfortable VR headset.",
        "price": 299.99
    },

    "AudioPhonic Noise-Canceling Headphones": {
        "name": "AudioPhonic Noise-Canceling Headphones",
        "category": "Audio Equipment",
        "brand": "AudioPhonic",
        "model_number": "AP-NC100",
        "warranty": "1 year",
        "rating": 4.6,
        "features": ["Active noise-canceling", "Bluetooth", "20-hour battery life", "Comfortable fit"],
        "description": "Experience immersive sound with these noise-canceling headphones.",
        "price": 199.99
    },
    "WaveSound Bluetooth Speaker": {
        "name": "WaveSound Bluetooth Speaker",
        "category": "Audio Equipment",
        "brand": "WaveSound",
        "model_number": "WS-BS50",
        "warranty": "1 year",
        "rating": 4.5,
        "features": ["Portable", "10-hour battery life", "Water-resistant", "Built-in microphone"],
        "description": "A compact and versatile Bluetooth speaker for music on the go.",
        "price": 49.99
    },
    "AudioPhonic True Wireless Earbuds": {
        "name": "AudioPhonic True Wireless Earbuds",
        "category": "Audio Equipment",
        "brand": "AudioPhonic",
        "model_number": "AP-TW20",
        "warranty": "1 year",
        "rating": 4.4,
        "features": ["True wireless", "Bluetooth 5.0", "Touch controls", "18-hour battery life"],
        "description": "Enjoy music without wires with these comfortable true wireless earbuds.",
        "price": 79.99
    },
    "WaveSound Soundbar": {
        "name": "WaveSound Soundbar",
        "category": "Audio Equipment",
        "brand": "WaveSound",
        "model_number": "WS-SB40",
        "warranty": "1 year",
        "rating": 4.3,
        "features": ["2.0 channel", "80W output", "Bluetooth", "Wall-mountable"],
        "description": "Upgrade your TV's audio with this slim and powerful soundbar.",
        "price": 99.99
    },
    "AudioPhonic Turntable": {
        "name": "AudioPhonic Turntable",
        "category": "Audio Equipment",
        "brand": "AudioPhonic",
        "model_number": "AP-TT10",
        "warranty": "1 year",
        "rating": 4.2,
        "features": ["3-speed", "Built-in speakers", "Bluetooth", "USB recording"],
        "description": "Rediscover your vinyl collection with this modern turntable.",
        "price": 149.99
    },

    "FotoSnap DSLR Camera": {
        "name": "FotoSnap DSLR Camera",
        "category": "Cameras and Camcorders",
        "brand": "FotoSnap",
        "model_number": "FS-DSLR200",
        "warranty": "1 year",
        "rating": 4.7,
        "features": ["24.2MP sensor", "1080p video", "3-inch LCD", "Interchangeable lenses"],
        "description": "Capture stunning photos and videos with this versatile DSLR camera.",
        "price": 599.99
    },
    "ActionCam 4K": {
        "name": "ActionCam 4K",
        "category": "Cameras and Camcorders",
        "brand": "ActionCam",
        "model_number": "AC-4K",
        "warranty": "1 year",
        "rating": 4.4,
        "features": ["4K video", "Waterproof", "Image stabilization", "Wi-Fi"],
        "description": "Record your adventures with this rugged and compact 4K action camera.",
        "price": 299.99
    },
    "FotoSnap Mirrorless Camera": {
        "name": "FotoSnap Mirrorless Camera",
        "category": "Cameras and Camcorders",
        "brand": "FotoSnap",
        "model_number": "FS-ML100",
        "warranty": "1 year",
        "rating": 4.6,
        "features": ["20.1MP sensor", "4K video", "3-inch touchscreen", "Interchangeable lenses"],
        "description": "A compact and lightweight mirrorless camera with advanced features.",
        "price": 799.99
    },
    "ZoomMaster Camcorder": {
        "name": "ZoomMaster Camcorder",
        "category": "Cameras and Camcorders",
        "brand": "ZoomMaster",
        "model_number": "ZM-CM50",
        "warranty": "1 year",
        "rating": 4.3,
        "features": ["1080p video", "30x optical zoom", "3-inch LCD", "Image stabilization"],
        "description": "Capture life's moments with this easy-to-use camcorder.",
        "price": 249.99
    },
    "FotoSnap Instant Camera": {
        "name": "FotoSnap Instant Camera",
        "category": "Cameras and Camcorders",
        "brand": "FotoSnap",
        "model_number": "FS-IC10",
        "warranty": "1 year",
        "rating": 4.1,
        "features": ["Instant prints", "Built-in flash", "Selfie mirror", "Battery-powered"],
        "description": "Create instant memories with this fun and portable instant camera.",
        "price": 69.99
    }
}
```

下面的的函数用于从上面的products数据中提取相应的产品信息。

```python
def get_product_by_name(name):
    return products.get(name, None)

def get_products_by_category(category):
    return [product for product in products.values() if product["category"] == category]
```

```python
import json 

def read_string_to_list(input_string):
    if input_string is None:
        return None

    try:
        input_string = input_string.replace("'", "\"")  # Replace single quotes with double quotes for valid JSON
        data = json.loads(input_string)
        return data
    except json.JSONDecodeError:
        print("Error: Invalid JSON string")
        return None   
    
def generate_output_string(data_list):
    output_string = ""

    if data_list is None:
        return output_string

    for data in data_list:
        try:
            if "products" in data:
                products_list = data["products"]
                for product_name in products_list:
                    product = get_product_by_name(product_name)
                    if product:
                        output_string += json.dumps(product, indent=4) + "\n"
                    else:
                        print(f"Error: Product '{product_name}' not found")
            elif "category" in data:
                category_name = data["category"]
                category_products = get_products_by_category(category_name)
                for product in category_products:
                    output_string += json.dumps(product, indent=4) + "\n"
            else:
                print("Error: Invalid object format")
        except Exception as e:
            print(f"Error: {e}")

    return output_string 
```



这里提取出产品的相关信息。

```python
category_and_product_list = read_string_to_list(category_and_product_response_1)
#print(category_and_product_list)

product_information_for_user_message_1 = generate_output_string(category_and_product_list)
print(product_information_for_user_message_1)
```

输出：

```python
{
    "name": "SmartX ProPhone",
    "category": "Smartphones and Accessories",
    "brand": "SmartX",
    "model_number": "SX-PP10",
    "warranty": "1 year",
    "rating": 4.6,
    "features": [
        "6.1-inch display",
        "128GB storage",
        "12MP dual camera",
        "5G"
    ],
    "description": "A powerful smartphone with advanced camera features.",
    "price": 899.99
}
{
    "name": "FotoSnap DSLR Camera",
    "category": "Cameras and Camcorders",
    "brand": "FotoSnap",
    "model_number": "FS-DSLR200",
    "warranty": "1 year",
    "rating": 4.7,
    "features": [
        "24.2MP sensor",
        "1080p video",
        "3-inch LCD",
        "Interchangeable lenses"
    ],
    "description": "Capture stunning photos and videos with this versatile DSLR camera.",
    "price": 599.99
}
{
    "name": "CineView 4K TV",
    "category": "Televisions and Home Theater Systems",
    "brand": "CineView",
    "model_number": "CV-4K55",
    "warranty": "2 years",
    "rating": 4.8,
    "features": [
        "55-inch display",
        "4K resolution",
        "HDR",
        "Smart TV"
    ],
    "description": "A stunning 4K TV with vibrant colors and smart features.",
    "price": 599.99
}
{
    "name": "SoundMax Home Theater",
    "category": "Televisions and Home Theater Systems",
    "brand": "SoundMax",
    "model_number": "SM-HT100",
    "warranty": "1 year",
    "rating": 4.4,
    "features": [
        "5.1 channel",
        "1000W output",
        "Wireless subwoofer",
        "Bluetooth"
    ],
    "description": "A powerful home theater system for an immersive audio experience.",
    "price": 399.99
}
{
    "name": "CineView 8K TV",
    "category": "Televisions and Home Theater Systems",
    "brand": "CineView",
    "model_number": "CV-8K65",
    "warranty": "2 years",
    "rating": 4.9,
    "features": [
        "65-inch display",
        "8K resolution",
        "HDR",
        "Smart TV"
    ],
    "description": "Experience the future of television with this stunning 8K TV.",
    "price": 2999.99
}
{
    "name": "SoundMax Soundbar",
    "category": "Televisions and Home Theater Systems",
    "brand": "SoundMax",
    "model_number": "SM-SB50",
    "warranty": "1 year",
    "rating": 4.3,
    "features": [
        "2.1 channel",
        "300W output",
        "Wireless subwoofer",
        "Bluetooth"
    ],
    "description": "Upgrade your TV's audio with this sleek and powerful soundbar.",
    "price": 199.99
}
{
    "name": "CineView OLED TV",
    "category": "Televisions and Home Theater Systems",
    "brand": "CineView",
    "model_number": "CV-OLED55",
    "warranty": "2 years",
    "rating": 4.7,
    "features": [
        "55-inch display",
        "4K resolution",
        "HDR",
        "Smart TV"
    ],
    "description": "Experience true blacks and vibrant colors with this OLED TV.",
    "price": 1499.99
}

```

到这里模型就有了回答用户提问所需要的产品信息了。



### 根据详细的产品信息，生成对用户查询的答案

```python
system_message = f"""
You are a customer service assistant for a \
large electronic store. \
Respond in a friendly and helpful tone, \
with very concise answers. \
Make sure to ask the user relevant follow up questions.
"""
user_message_1 = f"""
tell me about the smartx pro phone and \
the fotosnap camera, the dslr one. \
Also tell me about your tvs"""
messages =  [  
{'role':'system',
 'content': system_message},   
{'role':'user',
 'content': user_message_1},  
{'role':'assistant',
 'content': f"""Relevant product information:\n\
 {product_information_for_user_message_1}"""},   
]
final_response = get_completion_from_messages(messages)
print(final_response)
```

回答：

```python
The SmartX ProPhone is a powerful smartphone with a 6.1-inch display, 128GB storage, a 12MP dual camera, and 5G capability. It is priced at $899.99 and comes with a 1-year warranty. 

The FotoSnap DSLR Camera is a versatile camera with a 24.2MP sensor, 1080p video recording, a 3-inch LCD screen, and interchangeable lenses. It is priced at $599.99 and also comes with a 1-year warranty.

As for our TVs, we have a range of options. The CineView 4K TV is a 55-inch TV with 4K resolution, HDR, and smart TV features. It is priced at $599.99 and comes with a 2-year warranty.

We also have the CineView 8K TV, which is a 65-inch TV with 8K resolution, HDR, and smart TV features. It is priced at $2999.99 and also comes with a 2-year warranty.

Lastly, we have the CineView OLED TV, which is a 55-inch TV with 4K resolution, HDR, and smart TV features. It is priced at $1499.99 and comes with a 2-year warranty.

Is there anything specific you would like to know about these products?
```



我们有选择地将产品描述加载到提示词中，而不是包括所有的产品描述，只让模型使用它所需要的信息。



## 总结

为什么我们不把所有的产品信息包括在提示词中呢？这样我们就不必为所有这些中间步骤而烦恼，而直接去查询产品信息。



首先，包括所有的产品描述可能会使模型的上下文更加混乱，就像一个人试图一次处理大量的信息一样。

其次，语言模型有上下文限制。只允许固定数量的Token作为输入和输出。因此，如果你有大量的产品信息，你甚至无法将其装入一个prompt。

最后，包括所有的产品描述是昂贵的。处理那些不必要的信息也要算钱的。因此，通过有选择地加载信息，你可以减少生成式应用的成本。



一般来说，确定何时动态加载信息到模型中，并允许模型决定何时需要更多的信息，这是增强这些模型能力的最佳方式之一。



在这个例子中，我们只添加了对一个或多个特定函数的调用，以按产品名称获取产品描述，或按类别名称获取相应类别产品的信息。但是，模型实际上善于决定何时使用各种不同的工具，并能在指导下正确使用它们。而这就是ChatGPT插件的理念。我们告诉模型它可以使用哪些工具以及它们的作用，当它需要来自特定来源的信息或想采取一些其他适当的行动时，它就会选择使用这些工具。



另外，在这个例子中，我们只能通过精确的产品名称和类别名称的匹配来查询信息，但也有更高级的信息检索技术。检索信息最有效的方法之一是使用`text embedding`。`embedding`可用于在大型语料库上实现高效的知识检索，以找到与给定查询有关的信息。使用`embedding`的一个关键优势是，它能够实现模糊的语义搜索，这使你能够在不使用确切的关键词的情况下找到相关信息。这一部分以后再介绍。



参考：

[https://learn.deeplearning.ai/chatgpt-building-system/lesson/6/chaining-prompts](https://learn.deeplearning.ai/chatgpt-building-system/lesson/6/chaining-prompts)



文章中不好放全部的示例代码，公众号内回复 “api” 关键字可获取本篇文章完整的示例代码（格式为ipynb）。



---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)