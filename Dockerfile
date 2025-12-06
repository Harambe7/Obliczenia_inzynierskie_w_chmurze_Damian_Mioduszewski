FROM python:3.9-slim
RUN apt-get update && apt-get install -y vim

RUN pip install matplotlib
RUN pip install numpy

WORKDIR /home/tempomat
COPY . .

CMD ["python", "main.py"]