#!/bin/bash -e

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))
source $HERE/__init__.sh
ROOT=$(realpath $HERE/..)

source $ROOT/.anaconda3/bin/activate

echo_bold "==> Installing requirements with requirements.txt"
pip_install -r requirements.txt

echo_bold "==> Installing pre-commit"
pre-commit install

echo_bold "==> Installing mercury"
pip_install -e .

echo_bold "\nAll is well! You can start using this!

  $ source .anaconda3/bin/activate
"
