/**
 * drive_sys.xc
 *
 *  Created on: Jul 12, 2013
 *      Author: pkanajar
 */

/**
 *
 */
int update_statusword(int current_status, int status_reached) {
	return (current_status | status_reached);
}
