from os import listdir
from os.path import isfile, join
from sys import argv

def main(dir_path):
	files = [join(dir_path,f) for f in listdir(dir_path) if (isfile(join(dir_path, f)) and f.endswith('.frames'))]
	for f in sorted(files):
		print(f)

if __name__ == '__main__':
	if len(argv) < 2:
		print("Usage: python %s poses_directory" % argv[0])
		exit()

	main(argv[1])