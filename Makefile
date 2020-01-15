#simple make file
all:
	gcc AD4111_util.c -o AD4111_util -lpthread
clean:
	rm -f AD4111_util
