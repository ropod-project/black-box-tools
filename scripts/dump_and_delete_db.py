#!/usr/bin/env python3
import argparse

from black_box_tools.db_utils import DBUtils

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dump database (and optionally delete it afterwards)')
    parser.add_argument('-db', help='name of the mongo db', default='logs')
    parser.add_argument('-path', help='directory of the dump', default='.')
    parser.add_argument('-delete', help='name of the mongo db', default=True)

    args = parser.parse_args()
    db_name = args.db
    dump_path = args.path
    delete_db = args.delete

    print('Dumping database {0} to {1}'.format(db_name, dump_path))
    DBUtils.dump_db(db_name=db_name, data_dir=dump_path, delete_db=delete_db)
    print('{0} successfully dumped to {1}'.format(db_name, dump_path))
