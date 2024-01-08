import argparse
import os
from .compress.compress import BagCompress as bc
from .process.process import BagProcess as bp

def process(args):
    
    if args.batch is not None:
        bp.process_batch(args.batch,
                        args.dst,
                        args.name,
                        args.save_raw)
    elif args.folder is not None:
        bp.process_folder(args.folder,
                        args.dst,
                        args.name,
                        args.save_raw)

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

    # Create the parser for the "compress" command
    parser_compress = subparsers.add_parser('compress',
                                        help='compress help')
    parser_compress.add_argument('-b', '--batch', type=is_bag_dir,
                            help='path to a bag batch folder')
    parser_compress.add_argument('-f', '--folder', type=is_bag_dir,
                            help='path to a bag folder consisting bag batches')
    parser_compress.set_defaults(func=compress)
    
    # Create the parser for the "process" command
    parser_process = subparsers.add_parser('process',
                                        help='process help')
    parser_process.add_argument('-b', '--batch', type=is_bag_dir,
                            help='path to a bag batch folder')
    parser_process.add_argument('-f', '--folder', type=is_bag_dir,
                            help='path to a bag folder consisting bag batches')
    parser_process.add_argument('-d', '--dst',default=None,
                            help='path to a dataset destination folder')
    parser_process.add_argument('-n', '--name',default=None,
                            help='a custom name for the datafolder')
    parser_process.add_argument('-s', '--save_raw',action="store_true",
                            help='indicate if saving raw data or not')
    parser_process.set_defaults(func=process)

    # Parse the args and call the default function
    args = parser.parse_args()
    if hasattr(args, 'func'):
        args.func(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()