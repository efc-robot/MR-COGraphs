# MR-COGraphs
## Code
Coming soon!

## Dataset
### Isaac Small Environment

### Isaac Large Environment

### Replica Apartment2 Environment
single robot: https://cloud.tsinghua.edu.cn/f/e0ac84f0059142a48cd6/
two robots: https://cloud.tsinghua.edu.cn/f/960960e5dafc45fba511/?dl=1

### Real-world Environment


## Appendix

### How to train the encoder and decoder

1.Download the dataset from the following URL: https://www.kaggle.com/c/imagenet-object-localization-challenge/data.

2.Input the file LOC_synset_mapping.txt into a large language model (GPT/Kimi), with the prompt: "Based on the information in the dataset, each line begins with a serial number, followed by the word it represents. Select the serial numbers and words that will definitely appear in a room, and output the results in the format of 'serial number + word (reason)'."
The output file obtained is imagenet_classes_in_house_last.txt.

3.Utilizing the list of household objects from imagenet_classes_in_house_last.txt, in conjunction with the annotation files from ILSVRC/Annotations, obtain the cropped images.

4.Input the cropped images into CLIP to acquire features, which will subsequently be used for training the encoder and decoder.

### How to generate content for inquiry purposes
1.Ranking Node Labels by Frequency in COGraph: Sort the node labels in the COGraph dataset based on their occurrence frequency and select the top 10 labels, which will be referred to as "Appeared" in the Query Type.

2.Synonym Generation Using Large Language Models: Input the 10 labels identified in step 1 into a large language model (GPT/Kimi) with the prompt "Find synonyms for these words" to generate a list of synonyms for each label, denoted as "Similar" in the Query Type.

3.Descriptive Phrase Generation Using Large Language Models: Input the 10 labels from step 1 into a large language model (GPT/Kimi) with the prompt "Provide brief descriptions in English for these terms" to obtain a set of brief descriptions for each label, labeled as "Descriptive" in the Query Type.
