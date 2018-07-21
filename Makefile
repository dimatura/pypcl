develop:
	python setup.py develop

install:
	python setup.py install

build_ext:
	python setup.py build_ext

clean:
	rm -rf build pypcl.egg-info libpypcl.so
