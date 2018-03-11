'''
Created on 24 lut 2018

@author: arazu
'''

if __name__ == '__main__':
    pass

def simf(n1, n2 = 8): 
    answer = n1 + n2
    print('the answer is: ', answer)

# 
# text = 'sample text'
# saveFile = open('test.txt', 'w')
# saveFile.write(text)
# saveFile.close()

appendFile = open('test.txt', 'a')
appendFile.write('\nwarrupa boii?')
appendFile.close()
