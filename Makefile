#simple make file
all:
	gcc AD111_util.c -o AD4111CON -lpthread
clean:
	rm -f AD4111CON
