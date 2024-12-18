#!/bin/sh

# This script updates the online documentation on Github Pages via Doxygen
# and a seperate clone of the gh-pages branch.

REPOSITORY="https://github.com/KITmedical/kacanopen.git"

# Exit if any command fails.
set -e

command_exists(){
	[ -x "$(command -v $1)" ]
}

check_for_command() {
	if ! command_exists $1; then
		echo "ERROR: Please install $1." >&2
		exit 1
	fi
}

check_for_command "git"
check_for_command "doxygen"

SCRIPTDIR=$(dirname "$(readlink -f "$0")")
HTMLDIR="$SCRIPTDIR/../html"

if [ ! -d "$HTMLDIR/.git" ]; then
	echo "Repository is not yet initialized."
	echo "Cloning gh-pages branch into $HTMLDIR now."
	if [ -d "$HTMLDIR" ]; then
		read -p "$HTMLDIR already exists and needs to be removed. Type [y] to continue.`echo $'\n> '`" choice
		case "$choice" in 
			y|Y|yes|Yes|YES ) rm -rf "$HTMLDIR" ;;
			* ) echo "Aborting..."; exit 0 ;;
		esac
	fi
	git clone -b gh-pages "$REPOSITORY" "$HTMLDIR"
else
	echo "$HTMLDIR/.git exists, so I assume it already contains the gh-pages branch of the kacanopen repository."
fi

echo "Running doxygen now."
cd "$SCRIPTDIR/.."
doxygen Doxyfile

echo "Committing and pushing changes now."
cd "$HTMLDIR"
echo "> git pull"
git pull
echo "> git add *"
git add *
echo "> git commit -m \"Update via dev/update_online_docs.sh.\""
git commit -m "Update via dev/update_online_docs.sh."
echo "> git push"
git push
echo "Updating online docs finished."
