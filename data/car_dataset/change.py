import os

def readFile(path):
    with open(path, 'rt') as f:
        return f.read()
    

def loadBoardPath(pathname):
    boardPaths = []
    for filename in os.listdir(pathname):
        if filename.endswith('txt'):
            boardPaths.append(filename)
    return boardPaths

def writeFile(path, contents):
    with open(path, 'wt') as f:
        f.write(contents)

labelsList = loadBoardPath('valid/labels/')
for path in labelsList:
    pathname = 'valid/labels/' + path
    read_content = readFile(pathname)
    # print(read_content)
    # print(pathname)
    contents = []
    for line in read_content.splitlines():
        contents.append('3' + line[1:])
    # while True:
    #     line = read_content.readline()
    #     if not line:
    #         break
    #     contents.append('3' + line[1:] + '\n')
    # lines = read_content.readline()
    # contents = ''
    # for line in enumerate(lines):
    #     contents.append('3' + line[1:] + '\n')
    content = ''
    for item in contents:
        # print(item)
        content+= '3' + item[1:] +'\n'
    # print(content)
    writeFile(f'/home/cindy/car_dataset/valid/labels_final/{path}', content)