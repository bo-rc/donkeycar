import uff
import sys

if len(sys.argv) != 2:
    print('Usage:', sys.argv[0], ' /path/to/model.pb')
    exit()

print('Converting...')
filename = sys.argv[1]
o_filename = filename[:filename.rfind('.')]  + '.uff'
trt_graph = uff.from_tensorflow_frozen_model(filename, output_filename=o_filename)
print('Done')
