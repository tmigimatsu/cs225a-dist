#!/bin/bash

# CS327a HW2 submission script: Adapted from AA274 hw submission script :)

if [ "$PWD" != "$HOME/cs327a/scripts" ]; then
    read -p "Current directory ($PWD) is not the default location for cs327a/scripts/. Continue? [yn]: " yn
    case $yn in
        [Yy]* ) ;;
        * ) exit;;
    esac
fi

read -p "Please enter your SUNetID: " sunetid

rm -f "${sunetid}_hw2.zip"
echo "Creating ${sunetid}_hw2.zip"
zip -q "${sunetid}_hw2.zip" "submit_hw2.sh"
zip -qd "${sunetid}_hw2.zip" "submit_hw2.sh" # making an empty zip file

for fname in "../hw2/p1-main.cpp" \
             "../hw2/p2-main.cpp" \
             "../hw2/p3-main.cpp"
do
    if [ -f $fname ]; then
        zip "${sunetid}_hw2.zip" $fname
    else
        read -p "$fname not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
done

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password when prompted."
lastsub=$(ssh $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/cs327a/HW2_code 2>/dev/null | grep -m 1 ${sunetid}_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password when prompted."
set -x
scp "${sunetid}_hw2.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/cs327a/HW2_code/${sunetid}_submission_$subnum.zip" 2>/dev/null
