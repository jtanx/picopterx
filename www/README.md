Web interface
=============
This is the web interface. It is written in a mix of HTML, PHP, JavaScript and CSS, with a touch of Python tacked on at the end.

## Getting started
1. You must have a web server installed. We use lighttpd. You can probably use Apache/nginx if you want.
     ~~~~~
	 sudo apt-get install lighttpd php5-cgi 
	 sudo lighty-enable-mod fastcgi
	 sudo lighty-enable-mod fastcgi-php
	 ~~~~~

2. You must set the web root to be this folder. I've found it easiest to just symlink it to this so that changes are reflected immediately plus you get version control:

     ~~~~~
	 sudo rm -r /var/www
	 cd /var
	 sudo ln -s ~/picopterx/www
	 ~~~~~
	 
	 You will probably need to edit lighttpd's config file to use this location:
	 
	 ~~~~~
	 sudo nano /etc/lighttpd/lighttpd.conf
	 change document-root to be "/var/www"
	 ~~~~~
	 
	 See [here](https://github.com/jtanx/picopterx/blob/master/system/etc/lighttpd/lighttpd.conf) for an example.
3. You must manually modify `ajax-thrift.php` to point to the Thrift files that you have. Do not commit this to git as they need to be set like so for running on the RPi. Wherever you see `/home/pi/software/thrift-0.9.2` replace with your own path.
4. You need to get the map tiles and place them in the `tiles` folder (you must create this folder). You can download the tiles needed using a tile downloader, or you may get a copy off the RPi (*cough*).
5. To make the GPX downloader work, you must ensure that the PHP worker has write permissions to the log folder. While not the best security wise, I've found it easiest to just chmod the folder like so:
    ~~~~~
	chmod 777 ~/logs 
	~~~~~

6. Furthermore, you need to install the python dependencies:
    ~~~~~
	sudo apt-get install python python-pip
	sudo pip install gpxpy pytz numpy
	~~~~~

7. At this stage, the web interface *should* be working. If it's not, you can check the PHP error log:
    ~~~~~
	sudo cat /var/log/lighttpd/error.log
	or perhaps
	sudo tail /var/log/lighttpd/error.log
	~~~~~
