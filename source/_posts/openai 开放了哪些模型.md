---
title: openai 开放了哪些模型
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: d54d6e7a
date: 2023-09-30 20:03:16
---


```bash
%env OPENAI_API_KEY=sk-XXX
```

获取openai开放的模型列表：

```python
import openai
import pandas as pd
import os

openai.api_key = os.environ.get("OPENAI_API_KEY")
# list all open ai models
engines = openai.Engine.list()
pd = pd.DataFrame(openai.Engine.list()['data'])
display(pd[['id', 'owner']])
```
<!--more-->
结果：

| index | id                            | owner           |
| ----- | ----------------------------- | --------------- |
| 0     | whisper-1                     | openai-internal |
| 1     | babbage                       | openai          |
| 2     | davinci                       | openai          |
| 3     | text-davinci-edit-001         | openai          |
| 4     | text-davinci-003              | openai-internal |
| 5     | babbage-code-search-code      | openai-dev      |
| 6     | text-similarity-babbage-001   | openai-dev      |
| 7     | code-davinci-edit-001         | openai          |
| 8     | text-davinci-001              | openai          |
| 9     | ada                           | openai          |
| 10    | babbage-code-search-text      | openai-dev      |
| 11    | babbage-similarity            | openai-dev      |
| 12    | code-search-babbage-text-001  | openai-dev      |
| 13    | text-curie-001                | openai          |
| 14    | code-search-babbage-code-001  | openai-dev      |
| 15    | text-ada-001                  | openai          |
| 16    | text-embedding-ada-002        | openai-internal |
| 17    | text-similarity-ada-001       | openai-dev      |
| 18    | curie-instruct-beta           | openai          |
| 19    | ada-code-search-code          | openai-dev      |
| 20    | ada-similarity                | openai-dev      |
| 21    | code-search-ada-text-001      | openai-dev      |
| 22    | text-search-ada-query-001     | openai-dev      |
| 23    | davinci-search-document       | openai-dev      |
| 24    | ada-code-search-text          | openai-dev      |
| 25    | text-search-ada-doc-001       | openai-dev      |
| 26    | davinci-instruct-beta         | openai          |
| 27    | text-similarity-curie-001     | openai-dev      |
| 28    | code-search-ada-code-001      | openai-dev      |
| 29    | ada-search-query              | openai-dev      |
| 30    | text-search-davinci-query-001 | openai-dev      |
| 31    | curie-search-query            | openai-dev      |
| 32    | davinci-search-query          | openai-dev      |
| 33    | babbage-search-document       | openai-dev      |
| 34    | ada-search-document           | openai-dev      |
| 35    | text-search-curie-query-001   | openai-dev      |
| 36    | text-search-babbage-doc-001   | openai-dev      |
| 37    | gpt-3\.5-turbo                | openai          |
| 38    | curie-search-document         | openai-dev      |
| 39    | text-search-curie-doc-001     | openai-dev      |
| 40    | babbage-search-query          | openai-dev      |
| 41    | text-babbage-001              | openai          |
| 42    | text-search-davinci-doc-001   | openai-dev      |
| 43    | text-search-babbage-query-001 | openai-dev      |
| 44    | curie-similarity              | openai-dev      |
| 45    | gpt-3\.5-turbo-0301           | openai          |
| 46    | curie                         | openai          |
| 47    | text-similarity-davinci-001   | openai-dev      |
| 48    | text-davinci-002              | openai          |
| 49    | davinci-similarity            | openai-dev      |



输出结果里有 49 个模型。其实顾名思义，你就能够知道这些模型是用来干啥的。比如 text-similarity-babbage-001 肯定就是用来进行相似度匹配的，就会比较适合用在我们 02 讲介绍的零样本分类。而 text-search-davinci-doc-001 肯定就更适合用来搜索文档。尽管有些模型的名字标注了 openai-dev 或者 openai-internal，但是这些模型都是可以使用的。比如，我们在 02 讲里面调用 get_embedding 方法拿到向量，背后用的就是 text-similarity-davinci-001 模型，也是一个 openai-dev 的模型。不过，里面的很多模型都已经老旧了，实际上主要用的模型就是这几类。

GPT-4 家族的模型，包括 gpt-4 和 gpt-4-0314。使用的方式和 ChatGPT 的模型一样，其中带日期的模型表示是一个模型快照。也就是模型不会随着时间迁移不断更新。GPT-4 的模型现在还很昂贵，输入 1000 个 Token 需要 0.03 美分，生成 1000 个 Token 则需要 0.06 美分。一般呢，我都是拿它帮我写代码，准确率会比较高。

GPT-3.5 家族的模型，包括 ChatGPT 所使用的 gpt-3.5-turbo 或者 gpt-3.5-turbo-0301，以及 text-davinci-003 和 text-davinci-002 这两个模型。前者专门针对对话的形式进行了微调，并且价格便宜，无论输入输出，1000 个 Token 都只需要 0.002 美分。后两个里，003 的模型有一个特殊的功能，就是支持“插入文本”这个功能，我们稍后就讲。003 也是基于强化学习微调的，而 002 则是做了监督学习下的微调。text-davinci-003 和 002 模型比 3.5-turbo 要贵 10 倍，但是输出更稳定。你可以根据自己的需要来决定。

剩下的，则是 Ada、Babbage、Curie 以及 Davinci 这四个基础模型。只适合用于下达单轮的指令，不适合考虑复杂的上下文和进行逻辑推理。这四个模型按照首字母排序，价格越来越贵，效果越来越好。而且我们如果要微调一个属于自己的模型，也需要基于这四个基础模型。

最后则是 text-embedding-ada-002、text-similarity-ada-001 这些专门用途模型。一般来说，我们通过这个模型来获取 Embedding，再用在其他的机器学习模型的训练，或者语义相似度的比较上。

所有模型的名字都来自科学史上的名人。Ada 来自人类史上第一位程序员 Ada，她也是著名诗人拜伦的女儿。而 Babadge 则是设计了分析机的巴贝奇，巴贝奇分析机也被认为是现代计算机的前身。Curie 则是指居里夫人，Davinci 是指达芬奇。我们可以挑几个模型，试一下它们 Embedding 的维度数量，你就知道模型的尺寸本身就是不一样的了。

```python

from openai.embeddings_utils import get_embedding

text = "让我们来算算Embedding"

embedding_ada = get_embedding(text, engine="text-embedding-ada-002")
print("embedding-ada: ", len(embedding_ada))

similarity_ada = get_embedding(text, engine="text-similarity-ada-001")
print("similarity-ada: ", len(similarity_ada))

babbage_similarity = get_embedding(text, engine="babbage-similarity")
print("babbage-similarity: ", len(babbage_similarity))

babbage_search_query = get_embedding(text, engine="text-search-babbage-query-001")
print("search-babbage-query: ", len(babbage_search_query))

curie = get_embedding(text, engine="curie-similarity")
print("curie-similarity: ", len(curie))

davinci = get_embedding(text, engine="text-similarity-davinci-001")
print("davinci-similarity: ", len(davinci))
```

输出结果：

```python

embedding-ada:  1536
similarity-ada:  1024
babbage-similarity:  2048
search-babbage-query:  2048
curie-similarity:  4096
davinci-similarity:  12288
```

可以看到，最小的 ada-similarity 只有 1024 维，而最大的 davinci-similarity 则有 12288 维，所以它们的效果和价格不同也是可以理解的了。