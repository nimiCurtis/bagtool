import argparse
import os
from .compress.compress import BagCompress as bc

def compress(args):

    if args.batch is not None:
        bc.compress_batch(args.batch)

    elif args.folder is not None:
        bc.compress_folder(args.folder)

def is_bag_file(arg_bag_str: str) -> str:
    """"""
    # check file validation
    if os.path.isfile(arg_bag_str) and arg_bag_str.split('.')[-1]=='bag':
        return arg_bag_str
    else:
        msg = f"Given bag file {arg_bag_str} is not valid! "
        raise argparse.ArgumentTypeError(msg)

def is_bag_dir(arg_bag_str:str):
    # check dir validation
    if os.path.isdir(arg_bag_str):
        return arg_bag_str
    else:
        msg = f"Given bag directory {arg_bag_str} is not valid! "
        raise argparse.ArgumentTypeError(msg)

def main():
    # Create the top-level parser
    parser = argparse.ArgumentParser(prog='bagtool')
    subparsers = parser.add_subparsers(help='sub-command help')

    # Create the parser for the "record" command
    parser_record = subparsers.add_parser('compress',
                                        help='compress help')
    parser_record.add_argument('-b', '--batch', type=is_bag_dir,
                            help='path to a bag batch folder')
    parser_record.add_argument('-f', '--folder', type=is_bag_dir,
                            help='path to a bag folder consisting bag batches')
    parser_record.set_defaults(func=compress)


    # Parse the args and call the default function
    args = parser.parse_args()
    if hasattr(args, 'func'):
        args.func(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()