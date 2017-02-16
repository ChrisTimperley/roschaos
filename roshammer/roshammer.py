#!/usr/bin/python3
import argparse

def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers() 
    parser.add_argument('--version', action='version', version='0.0.1')

    args = parser.parse_args() 

if __name__ == "__main__":
    main()
