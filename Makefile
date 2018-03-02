TARGET = FEHTeamA1

GITBINARY := git
FEHURL := google.com
FIRMWAREREPO := fehproteusfirmware

all:
	@cd $(FIRMWAREREPO) && make all TARGET=$(TARGET)

deploy:
	@cd $(FIRMWAREREPO) && make deploy TARGET=$(TARGET)

clean:
	@cd $(FIRMWAREREPO) && make clean TARGET=$(TARGET)

run:
	@cd $(FIRMWAREREPO) && make run TARGET=$(TARGET)